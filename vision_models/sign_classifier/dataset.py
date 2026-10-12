"""
Synthetic training data generator for the competition's stage road signs
(spec Fig. 2: 60cm disc, red ring, white fill, black bold digit centered).

No real photos of the physical signs exist yet, but the sign's design is
fully specified, so a photo dataset is generated procedurally instead of
collected: render the known sign design, then apply heavy augmentation
(rotation/perspective skew, scale, blur, lighting, noise, JPEG artifacts,
partial occlusion, varied backgrounds) to approximate the gap between a
clean render and a real camera frame. This is the standard approach for
detecting a simple, fully-specified, standardized pattern with no existing
photo corpus (traffic-sign-style classifiers are commonly bootstrapped
this way before real data collection is possible).

Classes: 0 = "not a sign" (background/distractor patch, for rejecting
false-positive proposals from the classical red-ring localizer that feeds
this classifier - see sign_detector.py), 1-10 = stage number (stage 10
renders as the two-character string "10").

Real competition-photo data should replace/augment this once the vehicle
and printed signs are both available - see CLAUDE.md's hardware-validation
policy. Nothing trained here should be trusted against real signs until
tested against actual camera frames.
"""
import random
import string

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

IMG_SIZE = 64  # model input resolution
RENDER_SIZE = 160  # render at higher res, then downsample after augmentation
NUM_CLASSES = 11  # 0=none, 1..10=stage number

FONT_PATH = "C:/Windows/Fonts/arialbd.ttf"  # Arial Bold - closest system font to spec's "Arial Black"


def _random_color(low=0, high=255):
    return tuple(random.randint(low, high) for _ in range(3))


def _make_background(size):
    """A plausible non-sign background: gradient, barrier red/white stripe,
    dirt/gravel-ish noise texture, sky, or flat color - the parkour's actual
    visual context (red/white barriers, aggregate road, dirt) plus generic
    clutter so the classifier isn't just a red-blob detector."""
    kind = random.choice(["gradient", "stripe", "noise", "flat", "grass_dirt"])
    img = Image.new("RGB", (size, size))
    draw = ImageDraw.Draw(img)

    if kind == "flat":
        img.paste(_random_color(30, 220), [0, 0, size, size])

    elif kind == "gradient":
        c1, c2 = _random_color(20, 200), _random_color(20, 200)
        arr = np.linspace(0, 1, size).reshape(-1, 1, 1)
        grad = (np.array(c1) * (1 - arr) + np.array(c2) * arr).astype(np.uint8)
        grad = np.repeat(grad, size, axis=1)
        img = Image.fromarray(grad)

    elif kind == "stripe":
        # red/white parkour barrier - a real, likely false-positive source
        stripe_w = random.randint(10, 30)
        colors = [(200, 20, 20), (240, 240, 240)]
        for i in range(0, size * 2, stripe_w):
            draw.rectangle([i - size, 0, i - size + stripe_w, size], fill=colors[(i // stripe_w) % 2])
        img = img.rotate(random.uniform(-45, 45))

    elif kind == "grass_dirt":
        base = np.array(_random_color(60, 140))
        noise = np.random.normal(0, 25, (size, size, 3))
        arr = np.clip(base + noise, 0, 255).astype(np.uint8)
        img = Image.fromarray(arr)

    else:  # noise
        arr = np.random.randint(0, 255, (size, size, 3), dtype=np.uint8)
        img = Image.fromarray(arr)

    return img


def _render_sign(stage, size):
    """Clean render of the sign design for a given stage number (1-10)."""
    img = Image.new("RGB", (size, size), (0, 0, 0))
    draw = ImageDraw.Draw(img)
    cx, cy = size / 2, size / 2
    outer_r = size * random.uniform(0.42, 0.49)
    ring_frac = random.uniform(0.14, 0.22)  # ring thickness as a fraction of outer radius
    inner_r = outer_r * (1 - ring_frac)

    draw.ellipse([cx - outer_r, cy - outer_r, cx + outer_r, cy + outer_r], fill=(200, 20, 20))
    draw.ellipse([cx - inner_r, cy - inner_r, cx + inner_r, cy + inner_r], fill=(245, 245, 245))

    text = str(stage)
    font_size = int(inner_r * (1.35 if len(text) == 1 else 1.05))
    try:
        font = ImageFont.truetype(FONT_PATH, font_size)
    except OSError:
        font = ImageFont.load_default()
    bbox = draw.textbbox((0, 0), text, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text((cx - tw / 2 - bbox[0], cy - th / 2 - bbox[1]), text, fill=(10, 10, 10), font=font)

    return img, (cx, cy, outer_r)


def _random_distractor(size):
    """A non-sign shape that could fool a color/circularity-only detector:
    a plain colored disc, a partial ring, or a red blob with no digit."""
    img = _make_background(size)
    draw = ImageDraw.Draw(img)
    if random.random() < 0.6:
        cx, cy = size / 2 + random.uniform(-10, 10), size / 2 + random.uniform(-10, 10)
        r = size * random.uniform(0.2, 0.48)
        color = random.choice([(200, 20, 20), _random_color(), (245, 245, 245)])
        bbox = [cx - r, cy - r, cx + r, cy + r]
        if random.random() < 0.5:
            draw.ellipse(bbox, fill=color)
        else:
            start, end = sorted(random.sample(range(360), 2))
            draw.pieslice(bbox, start, end, fill=color)
    return img


def _augment(img):
    """Photometric + geometric augmentation to bridge clean-render -> camera-frame."""
    # Rotation (camera roll / sign not perfectly upright)
    img = img.rotate(random.uniform(-20, 20), resample=Image.BICUBIC, fillcolor=_random_color())

    # Perspective-ish skew via affine shear (off-axis viewing angle)
    if random.random() < 0.6:
        shear = random.uniform(-0.25, 0.25)
        w, h = img.size
        img = img.transform((w, h), Image.AFFINE, (1, shear, 0, 0, 1, 0), resample=Image.BICUBIC,
                             fillcolor=_random_color())

    # Blur (defocus or motion)
    if random.random() < 0.5:
        img = img.filter(ImageFilter.GaussianBlur(random.uniform(0.5, 2.5)))

    arr = np.asarray(img).astype(np.float32)

    # Brightness / contrast
    brightness = random.uniform(0.5, 1.6)
    contrast = random.uniform(0.7, 1.4)
    arr = (arr - 128) * contrast + 128 * brightness
    arr = np.clip(arr, 0, 255)

    # Gaussian sensor noise
    if random.random() < 0.7:
        arr = np.clip(arr + np.random.normal(0, random.uniform(2, 15), arr.shape), 0, 255)

    img = Image.fromarray(arr.astype(np.uint8))

    # Partial occlusion (glare, another object clipping the sign edge)
    if random.random() < 0.25:
        draw = ImageDraw.Draw(img)
        w, h = img.size
        x0, y0 = random.uniform(0, w), random.uniform(0, h)
        x1, y1 = x0 + random.uniform(10, w * 0.5), y0 + random.uniform(10, h * 0.5)
        draw.rectangle([x0, y0, x1, y1], fill=_random_color())

    return img


def generate_sample(rng=None):
    """Returns (HWC uint8 RGB array at IMG_SIZE, class label 0-10)."""
    if rng is not None:
        random.seed(rng)
        np.random.seed(rng)

    label = random.randint(0, NUM_CLASSES - 1)

    if label == 0:
        img = _random_distractor(RENDER_SIZE) if random.random() < 0.5 else _make_background(RENDER_SIZE)
    else:
        bg = _make_background(RENDER_SIZE)
        sign, (cx, cy, r) = _render_sign(label, RENDER_SIZE)
        # composite sign onto background using its circular alpha mask, with
        # a random scale/position so the sign doesn't always fill the frame
        mask = Image.new("L", (RENDER_SIZE, RENDER_SIZE), 0)
        ImageDraw.Draw(mask).ellipse([cx - r, cy - r, cx + r, cy + r], fill=255)
        scale = random.uniform(0.55, 1.0)
        new_size = int(RENDER_SIZE * scale)
        sign_s = sign.resize((new_size, new_size))
        mask_s = mask.resize((new_size, new_size))
        off = ((RENDER_SIZE - new_size) // 2 + random.randint(-15, 15),
               (RENDER_SIZE - new_size) // 2 + random.randint(-15, 15))
        img = bg.copy()
        img.paste(sign_s, off, mask_s)

    img = _augment(img)
    img = img.resize((IMG_SIZE, IMG_SIZE), Image.BICUBIC)
    return np.asarray(img, dtype=np.uint8), label
