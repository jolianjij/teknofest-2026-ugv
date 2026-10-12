"""
Classical (non-learned) drivable-path segmentation, usable standalone with
NO training data and NO trained model. This exists for two reasons:

1. It's a real, deployable fallback - unlike the sign (a fully known, fixed
   geometric pattern that can be synthesised photorealistically), off-road
   terrain appearance can't be faked, so there is no synthetic-data shortcut
   here. A CNN (see cnn_model.py) can only be meaningfully trained once real
   front-camera frames exist.
2. It is also the pseudo-label source for training that CNN: run this over
   captured frames to auto-generate drivable/non-drivable masks with no
   manual annotation, then train on those. Bootstrapping a fast CNN from a
   slower classical heuristic's own output is well-established for
   ground-plane/terrain classification (DARPA LAGR-era self-supervised
   terrain classifiers did exactly this).

Algorithm: assume a small seed region directly in front of the vehicle
(bottom-centre of frame) is currently-drivable terrain - a safe assumption
for any frame captured while the vehicle is on-course. Measure that
region's colour statistics in Lab, mark every pixel close to them, then
keep only the connected region that actually touches the seed.

IMPLEMENTATION NOTE - no ximgproc. An earlier version used SLIC
superpixels from cv2.ximgproc, but the Orange Pi's OpenCV 4.11 build has
no contrib modules, so that version raised ImportError on the target
hardware - i.e. the "always available" fallback was not available where it
actually had to run. Every operation below is base-OpenCV. Edge-preserving
blur plus morphology gives the spatial regularisation the superpixels were
providing, and the connected-component step is a genuine improvement: it
removes colour-matching patches that are not contiguous with the ground the
vehicle is standing on (sky through a doorway, a similarly-coloured wall).

This remains a WEAK colour-similarity heuristic. It will struggle with
strong lighting gradients, shadows, and terrain that legitimately changes
colour while staying drivable. It is a preference/pseudo-label signal only:
per CLAUDE.md's vision strategy, lidar stays authoritative for lethal
obstacle avoidance regardless of what this produces.
"""
import cv2
import numpy as np

SEED_WIDTH_FRAC = 0.3    # fraction of image width, centred
SEED_HEIGHT_FRAC = 0.15  # fraction of image height, up from the bottom
COLOR_DIST_THRESHOLD = 16.0  # Lab distance floor; tune against real footage
MAD_SCALE = 3.0          # accept within this many robust deviations of the seed
MAX_ADAPTIVE_DIST = 34.0  # hard ceiling on the adaptive term - see note below
L_WEIGHT = 0.35          # de-weight luminance relative to chroma
MORPH_KERNEL = np.ones((7, 7), np.uint8)
WORK_WIDTH = 320         # segment at reduced width for speed, then upscale


def _seed_mask(shape):
    h, w = shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    sw, sh = int(w * SEED_WIDTH_FRAC), int(h * SEED_HEIGHT_FRAC)
    x0, y0 = (w - sw) // 2, h - sh
    mask[y0:h, x0:x0 + sw] = 1
    return mask


def segment_drivable(frame_bgr):
    """Returns a uint8 HxW mask (255=drivable, 0=not) for one BGR frame."""
    if frame_bgr is None or frame_bgr.size == 0:
        return np.zeros((0, 0), dtype=np.uint8)

    full_h, full_w = frame_bgr.shape[:2]
    scale = WORK_WIDTH / float(full_w)
    small = cv2.resize(frame_bgr, (WORK_WIDTH, max(1, int(full_h * scale))),
                       interpolation=cv2.INTER_AREA)

    # Edge-preserving smoothing: flattens texture within a surface without
    # bleeding across the boundary between surfaces.
    smooth = cv2.bilateralFilter(small, 7, 60, 60)
    lab = cv2.cvtColor(smooth, cv2.COLOR_BGR2LAB).astype(np.float32)

    seed = _seed_mask(small.shape).astype(bool)
    if not seed.any():
        return np.zeros((full_h, full_w), dtype=np.uint8)

    seed_pixels = lab[seed]
    # Median + MAD rather than mean + std: a blown-out highlight or a dark
    # shadow inside the seed box would otherwise dominate the statistics. That
    # is not hypothetical - on the first real frames the seed contained an
    # overexposed lamp, the std exploded, and the adaptive threshold below
    # accepted 77% of the image including walls and ceiling.
    seed_median = np.median(seed_pixels, axis=0)
    mad = np.median(np.abs(seed_pixels - seed_median), axis=0) * 1.4826

    # Weight luminance down: shadows and exposure shifts move L a lot while
    # leaving chroma roughly intact, so chroma is the more reliable evidence
    # that two patches are the same physical surface.
    weights = np.array([L_WEIGHT, 1.0, 1.0], dtype=np.float32)
    delta = (lab - seed_median) * weights
    distance = np.linalg.norm(delta, axis=2)

    # Whichever is more permissive: a fixed floor (uniform tarmac has almost no
    # spread) or the seed's own robust spread (textured gravel has plenty) -
    # but capped, so a pathological seed cannot make everything drivable.
    adaptive = min(MAX_ADAPTIVE_DIST, MAD_SCALE * float(np.linalg.norm(mad * weights)))
    threshold = max(COLOR_DIST_THRESHOLD, adaptive)
    mask = (distance < threshold).astype(np.uint8) * 255

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)

    # Keep only terrain continuous with the ground under the vehicle. A patch
    # that merely matches in colour but is disconnected from the seed is not
    # somewhere the vehicle can reach.
    mask = _largest_component_touching_seed(mask, seed)

    return cv2.resize(mask, (full_w, full_h), interpolation=cv2.INTER_NEAREST)


def _largest_component_touching_seed(mask, seed):
    count, labels = cv2.connectedComponents((mask > 0).astype(np.uint8), connectivity=8)
    if count <= 1:
        return mask
    seed_labels = labels[seed]
    seed_labels = seed_labels[seed_labels > 0]
    if seed_labels.size == 0:
        return np.zeros_like(mask)
    keep = np.bincount(seed_labels).argmax()
    return np.where(labels == keep, 255, 0).astype(np.uint8)


def overlay_preview(frame_bgr, mask):
    """Debug helper: green tint over the predicted drivable area."""
    overlay = frame_bgr.copy()
    sel = mask > 0
    if sel.any():
        overlay[sel] = (0.5 * overlay[sel] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
    return overlay
