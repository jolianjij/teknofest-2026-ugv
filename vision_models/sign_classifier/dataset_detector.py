"""
Synthetic training data for a sign DETECTOR (localizes + classifies in one
model, no color thresholding anywhere) - supersedes the
localizer(classical HSV)+classifier(CNN) split in dataset.py/sign_detector.py
per explicit request: detection itself must not depend on color.

Each sample is a full scene (not a pre-cropped ROI) containing either one
sign (at a random position/scale/rotation) or nothing, with:
  - has_object: bool
  - class_label: 1-10 (only meaningful if has_object)
  - bbox: (cx, cy, w, h) normalized to [0,1] of the full image (only
    meaningful if has_object)

To get an accurate bbox cheaply (no rotation-matrix bookkeeping), geometric
augmentation (rotation, shear) is applied to the SIGN+ALPHA-MASK patch
before it's composited onto the background - the bbox is then just the
transformed mask's own nonzero extent, measured directly rather than
computed analytically. Photometric augmentation (blur, brightness, noise,
occlusion) is applied to the whole composited scene afterward and doesn't
move anything, so the bbox stays valid.
"""
import gc
import random

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from dataset import _render_sign, _make_background, _random_color, FONT_PATH  # reuse existing helpers

IMG_SIZE = 128
NUM_STAGE_CLASSES = 10  # labels 1-10; "has_object" is a separate flag, no class-0 slot needed here

_sample_counter = 0
_GC_EVERY = 200  # defensive: this generator churns many mid-size PIL images per sample; training
                 # hit Windows/Pillow heap-fragmentation MemoryErrors over a long single-process run
                 # at the larger render_size this used to default to - periodic forced collection
                 # plus the smaller default size below avoids it


def _augment_sign_patch(sign_img, mask_img):
    """Rotate/shear the sign+mask together, return the transformed pair."""
    angle = random.uniform(-25, 25)
    sign_img = sign_img.rotate(angle, resample=Image.BICUBIC, fillcolor=(0, 0, 0))
    mask_img = mask_img.rotate(angle, resample=Image.NEAREST, fillcolor=0)

    if random.random() < 0.6:
        shear = random.uniform(-0.25, 0.25)
        w, h = sign_img.size
        sign_img = sign_img.transform((w, h), Image.AFFINE, (1, shear, 0, 0, 1, 0),
                                       resample=Image.BICUBIC, fillcolor=(0, 0, 0))
        mask_img = mask_img.transform((w, h), Image.AFFINE, (1, shear, 0, 0, 1, 0),
                                       resample=Image.NEAREST, fillcolor=0)
    return sign_img, mask_img


def _mask_bbox(mask_arr):
    """Tight bbox (x0,y0,x1,y1) of nonzero pixels, or None if empty."""
    ys, xs = np.where(mask_arr > 0)
    if xs.size == 0:
        return None
    return xs.min(), ys.min(), xs.max() + 1, ys.max() + 1


def _photometric_augment(img):
    if random.random() < 0.5:
        img = img.filter(ImageFilter.GaussianBlur(random.uniform(0.5, 2.0)))
    arr = np.asarray(img).astype(np.float32)
    brightness = random.uniform(0.6, 1.5)
    contrast = random.uniform(0.75, 1.3)
    arr = (arr - 128) * contrast + 128 * brightness
    if random.random() < 0.7:
        arr = arr + np.random.normal(0, random.uniform(2, 12), arr.shape)
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    img = Image.fromarray(arr)

    if random.random() < 0.2:
        draw = ImageDraw.Draw(img)
        w, h = img.size
        x0, y0 = random.uniform(0, w), random.uniform(0, h)
        x1, y1 = x0 + random.uniform(10, w * 0.3), y0 + random.uniform(10, h * 0.3)
        draw.rectangle([x0, y0, x1, y1], fill=_random_color())
    return img


def generate_detection_sample(rng=None, render_size=160, output_size=None):
    global _sample_counter
    _sample_counter += 1
    if _sample_counter % _GC_EVERY == 0:
        gc.collect()

    if rng is not None:
        random.seed(rng)
        np.random.seed(rng)

    scene = _make_background(render_size)
    has_object = random.random() < 0.75  # bias toward positives - negatives are easy, positives need the signal

    bbox_norm = (0.0, 0.0, 0.0, 0.0)
    class_label = 0

    if has_object:
        class_label = random.randint(1, NUM_STAGE_CLASSES)
        sign_size = random.randint(int(render_size * 0.25), int(render_size * 0.65))
        sign_img, (cx, cy, r) = _render_sign(class_label, sign_size)
        mask_img = Image.new("L", (sign_size, sign_size), 0)
        ImageDraw.Draw(mask_img).ellipse([cx - r, cy - r, cx + r, cy + r], fill=255)

        sign_img, mask_img = _augment_sign_patch(sign_img, mask_img)
        mask_arr = np.asarray(mask_img)
        bbox = _mask_bbox(mask_arr)

        if bbox is not None:
            x0, y0, x1, y1 = bbox
            max_x_off = render_size - sign_size
            off_x = random.randint(0, max(0, max_x_off))
            off_y = random.randint(0, max(0, max_x_off))
            scene.paste(sign_img, (off_x, off_y), mask_img)

            gx0, gy0, gx1, gy1 = x0 + off_x, y0 + off_y, x1 + off_x, y1 + off_y
            bbox_norm = (
                (gx0 + gx1) / 2 / render_size,
                (gy0 + gy1) / 2 / render_size,
                (gx1 - gx0) / render_size,
                (gy1 - gy0) / render_size,
            )
        else:
            has_object = False
            class_label = 0

    scene = _photometric_augment(scene)
    target_size = output_size if output_size is not None else IMG_SIZE
    scene = scene.resize((target_size, target_size), Image.BICUBIC)

    return np.asarray(scene, dtype=np.uint8), has_object, class_label, bbox_norm
