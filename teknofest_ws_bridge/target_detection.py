"""
Concentric-ring target detection, filtering and control.

ROS-independent so it can be tested without a robot or a camera.

Ported from the team's proven standalone tracker (target.py). Its detection
is markedly better than the Hough-circle approach it replaces, and the
reason is structural: concentric rings ARE a nested contour hierarchy, so
walking parent->child in `RETR_TREE` tests the thing the target actually
is. Hough needs radius hints, is sensitive to param2, degrades badly at low
resolution, and reports centres that scatter with radius - the previous
implementation scored 0/7 on clean synthetic targets because its fixed
concentricity tolerance could not group rings whose Hough centres sat 29 px
apart.

Two further ideas carried over from that tracker:
  * Both Otsu polarities are tried. The target is black/white/black, so
    whether the rings come out as nested contours depends on which way the
    threshold runs; trying both removes the guess.
  * CLAHE before thresholding, so a target in shade and a target in glare
    both reach Otsu with usable local contrast.
"""
from __future__ import annotations

import time
from collections import deque

import cv2
import numpy as np

# --- detection ---
# Range note (2026-09-09): the spec target must be engageable from 10m, but
# detection was only working out to ~5m. At the aim camera's configured
# 2560x1440 (camera_manager_node.py's PROFILES), the target IS theoretically
# resolvable at 10m - 85px across, ~14px inner-ring radius - so this was a
# detection-robustness problem, not a resolution ceiling. Every measurement
# this module makes (circularity, radius ratios) gets intrinsically noisier
# on a smaller contour: a +/-1px boundary/quantization error is a much
# bigger fraction of a 14px radius than a 28px one (roughly a 5m target).
# The thresholds below were tuned and validated against closer-range
# footage where that noise barely mattered - loosened here with that
# specifically in mind. UNVALIDATED against real 10m footage (no hardware
# access at the time of this change) - re-run the same detect-rate
# methodology this module was originally validated with, at 10m, before
# trusting these numbers; tighten back up if false-positive rate rises
# instead of the detect rate improving.
DETECT_MIN_AREA = 60
DETECT_MIN_CHAIN_LEN = 2
CIRCULARITY_MIN = 0.35         # was 0.45
MIN_DETECT_CONFIDENCE = 0.58   # was 0.70; every true detection measured >=0.93 at close range, expect lower at 10m
# Bounding-box width/height ratio ceiling (2026-09-10, noise-rejection pass -
# see _contour_metrics). A real ring is close to 1:1; this catches elongated
# blobs that a loosened CIRCULARITY_MIN alone might not, at zero cost to
# range sensitivity since a real target's aspect ratio doesn't change with
# distance.
MAX_ASPECT_RATIO = 1.4
# An outermost contour may not fill the frame. Under THRESH_BINARY the whole
# background becomes a single frame-sized contour with the target nested
# inside it; a rectangle's circularity is ~0.64, so it passes the roundness
# test and is accepted as the outer "ring". The chain's weighted centre then
# gets dragged toward image centre - measured 31 px of error on an
# off-centre target, which at 10 m is a miss.
MAX_OUTER_AREA_FRAC = 0.55
# The official target's radii are 6/12/18 cm - a clean 1:2:3. Nested contours
# that are round and concentric but do NOT sit in that proportion are not this
# target. Without this test, descending into background children to find
# off-centre targets also finds chains in random noise (measured: false
# positives on 6 of 7 non-target scenes). Checking the ratio is what makes the
# detector specific to the thing we are actually shooting at.
RING_RATIOS = (1.0, 2.0, 3.0)
RATIO_TOLERANCE = 0.35   # was 0.28 - see the range note above
MIN_RATIO_SCORE = 0.45   # was 0.55

# --- filter ---
FILTER_ALPHA = 0.35
FILTER_MAX_JUMP_PX = 60
FILTER_MAX_MISS_SECONDS = 0.6

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
# tileGridSize shrunk 8x8 -> 16x16 (range note above): at full 2560x1440 an
# 8x8 grid gives ~320x180px tiles - larger than the entire target (~85px)
# at 10m, so local contrast enhancement was being computed over an area
# dominated by background, diluting exactly the enhancement a small distant
# target needs. 16x16 gives ~160x90px tiles, closer to the target's own
# scale.
CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16))


def _ratio_score(chain):
    """How well a chain's radii match the target's 1:2:3 ring proportions.

    Radii are normalised by the smallest ring, then each is matched to its
    nearest expected ratio. Returns 0..1.
    """
    radii = sorted(m[2] for m in chain)
    if not radii or radii[0] <= 0:
        return 0.0
    normalised = [r / radii[0] for r in radii]
    scores = []
    for value in normalised:
        nearest = min(RING_RATIOS, key=lambda t: abs(value - t))
        # Tolerance scales with the ratio: the outer ring is allowed more
        # absolute slack than the inner one, which is where the pixels are.
        error = abs(value - nearest) / (RATIO_TOLERANCE * nearest)
        scores.append(max(0.0, 1.0 - error))
    return float(np.mean(scores))


def _contour_metrics(contour):
    area = cv2.contourArea(contour)
    if area < 1:
        return None
    # Aspect-ratio gate (2026-09-10, part of the noise-rejection pass):
    # circularity (area vs. enclosing-circle area) alone can still pass an
    # elongated blob once CIRCULARITY_MIN was loosened for 10m-range
    # detection - a real ring's bounding box is close to square regardless
    # of range, so this is a cheap, scale-independent extra check that
    # doesn't cost any of the range sensitivity CIRCULARITY_MIN gave up.
    _, _, bw, bh = cv2.boundingRect(contour)
    if bw == 0 or bh == 0:
        return None
    aspect = bw / bh if bw >= bh else bh / bw
    if aspect > MAX_ASPECT_RATIO:
        return None
    (x, y), r = cv2.minEnclosingCircle(contour)
    circle_area = np.pi * r * r
    if circle_area == 0:
        return None
    return x, y, r, area / circle_area, area


def _walk_concentric_chain(idx, contours, hierarchy,
                           center_tol_frac=0.30, min_radius_ratio=0.15):
    """Walk the nested contour chain from outer contour `idx` inward.

    Each child must be concentric with its parent, strictly smaller, and not
    vanishingly small - which is exactly the definition of a ring set and
    rejects nested noise that merely happens to be inside something round.
    """
    chain = []
    metrics = _contour_metrics(contours[idx])
    if metrics is None or metrics[3] < CIRCULARITY_MIN:
        return chain
    chain.append(metrics)

    child = hierarchy[0][idx][2]
    while child != -1:
        m = _contour_metrics(contours[child])
        if m is None or m[3] < CIRCULARITY_MIN:
            break
        px, py, pr, _, _ = chain[-1]
        cx, cy, cr, _, _ = m
        if np.hypot(cx - px, cy - py) > max(3.0, center_tol_frac * pr):
            break
        if cr >= pr * 0.95:
            break
        if cr < pr * min_radius_ratio:
            break
        chain.append(m)
        child = hierarchy[0][child][2]
    return chain


def detect_target(frame_bgr, min_area=DETECT_MIN_AREA,
                  min_chain_len=DETECT_MIN_CHAIN_LEN):
    """Returns (cx, cy, outer_radius, confidence) or None."""
    if frame_bgr is None or frame_bgr.size == 0:
        return None
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr
    gray = CLAHE.apply(gray)
    # 3x3, was 5x5 (range note above): a fixed-pixel blur kernel is a much
    # heavier relative smoothing pass on a 14px-radius ring than a 28px one -
    # at 10m it risks blurring adjacent rings into each other. Still enough
    # to fight sensor noise; if closer-range detection regresses, this is
    # the first thing to revert.
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    best_chain, best_confidence = [], -1.0

    for polarity in (cv2.THRESH_BINARY_INV, cv2.THRESH_BINARY):
        _, thresh = cv2.threshold(blur, 0, 255, polarity + cv2.THRESH_OTSU)
        proc = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=1)
        contours, hierarchy = cv2.findContours(proc, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours or hierarchy is None:
            continue

        frame_area = float(gray.shape[0] * gray.shape[1])
        for i, contour in enumerate(contours):
            if hierarchy[0][i][3] != -1:
                continue  # outermost only; children are reached by the walk
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            if area > MAX_OUTER_AREA_FRAC * frame_area:
                # Background blob, not a ring. Its children may still be a
                # real target, so descend to them instead of discarding.
                child = hierarchy[0][i][2]
                while child != -1:
                    child_chain = _walk_concentric_chain(child, contours, hierarchy)
                    ratio = _ratio_score(child_chain) if child_chain else 0.0
                    if len(child_chain) >= min_chain_len and ratio >= MIN_RATIO_SCORE:
                        avg_c = float(np.mean([m[3] for m in child_chain]))
                        depth = min(1.0, (len(child_chain) - 1) / 2.0)
                        conf = float(np.clip(
                            0.35 * avg_c + 0.30 * depth + 0.35 * ratio, 0.0, 1.0))
                        if len(child_chain) > len(best_chain) or (
                            len(child_chain) == len(best_chain) and conf > best_confidence
                        ):
                            best_chain, best_confidence = child_chain, conf
                    child = hierarchy[0][child][0]  # next sibling
                continue
            chain = _walk_concentric_chain(i, contours, hierarchy)
            if len(chain) < min_chain_len:
                continue

            ratio_score = _ratio_score(chain)
            if ratio_score < MIN_RATIO_SCORE:
                continue
            avg_circularity = float(np.mean([m[3] for m in chain]))
            depth_bonus = min(1.0, (len(chain) - 1) / 2.0)
            confidence = float(np.clip(
                0.35 * avg_circularity + 0.30 * depth_bonus + 0.35 * ratio_score, 0.0, 1.0))

            deeper = len(chain) > len(best_chain)
            same_depth_better = len(chain) == len(best_chain) and confidence > best_confidence
            if deeper or same_depth_better:
                best_chain, best_confidence = chain, confidence

    if len(best_chain) < min_chain_len or best_confidence < MIN_DETECT_CONFIDENCE:
        return None

    # Outer rings are larger and localise the centre more stably, so weight
    # the chain from the outside in.
    weights = np.array([len(best_chain) - k for k in range(len(best_chain))], dtype=float)
    cx = float(np.average([m[0] for m in best_chain], weights=weights))
    cy = float(np.average([m[1] for m in best_chain], weights=weights))
    return int(cx), int(cy), int(best_chain[0][2]), best_confidence


class TargetFilter:
    """Exponential smoothing with jump rejection and a miss timeout.

    A single frame that lands on a reflection would otherwise yank the aim
    off target; a jump beyond `max_jump_px` is treated as a miss instead,
    and the previous estimate is held until the target has genuinely been
    absent for `max_miss_seconds`.
    """

    def __init__(self, alpha=FILTER_ALPHA, max_jump_px=FILTER_MAX_JUMP_PX,
                 max_miss_seconds=FILTER_MAX_MISS_SECONDS):
        self.alpha = alpha
        self.max_jump_px = max_jump_px
        self.max_miss_seconds = max_miss_seconds
        self.center = None
        self.miss_since = None

    def update(self, detection, now=None):
        now = time.time() if now is None else now

        if detection is None:
            if self.miss_since is None:
                self.miss_since = now
            elif self.center is not None and (now - self.miss_since) > self.max_miss_seconds:
                self.center = None
            return self.center

        cx, cy, r = detection[0], detection[1], detection[2]
        if self.center is None:
            self.center = (float(cx), float(cy), float(r))
            self.miss_since = None
            return self.center

        was_mid_gap = self.miss_since is not None
        px, py, pr = self.center
        if np.hypot(cx - px, cy - py) > self.max_jump_px and not was_mid_gap:
            self.miss_since = now
            return self.center

        a = self.alpha
        self.center = (a * cx + (1 - a) * px, a * cy + (1 - a) * py, a * r + (1 - a) * pr)
        self.miss_since = None
        return self.center

    def reset(self):
        self.center = None
        self.miss_since = None


# ---- acquisition confirmation ----
# Continuous-position analogue of ugv_stage_sign_detector's TemporalConfirmer
# (that project's own noise-rejection design, validated on 4,300 real
# frames) - a sliding window requiring a majority of recent frames to agree
# before confirming anything, adapted here via spatial clustering (do
# enough recent detections fall within a small pixel radius of each other)
# since target position is continuous, not a discrete label like a sign ID.
# A lone false-positive - glare, motion blur, some other round object in
# the room - is very unlikely to reappear at the same pixel position
# several frames running the way a real, physically-present target will.
# Belongs here, not in aim_tracker.py, for the same reason TargetFilter/PID
# do: this class has no ROS dependency and needs none to be testable (see
# test_aim_pipeline.py).
ACQUIRE_HISTORY = 6
ACQUIRE_REQUIRED_HITS = 4
ACQUIRE_POSITION_TOLERANCE_PX = 30


class AcquisitionConfirmer:
    def __init__(self, history=ACQUIRE_HISTORY, required_hits=ACQUIRE_REQUIRED_HITS,
                 tolerance_px=ACQUIRE_POSITION_TOLERANCE_PX):
        self.history = deque(maxlen=history)
        self.required_hits = required_hits
        self.tolerance_px = tolerance_px
        self.last_hit_count = 0

    def update(self, detection):
        """detection: (cx, cy, radius) or None. Returns the confirmed
        (cx, cy, radius) once enough recent frames cluster together, else
        None. self.last_hit_count (for status/debug display) tracks how
        many frames currently support the best cluster."""
        self.history.append(detection)
        candidates = [d for d in self.history if d is not None]
        self.last_hit_count = 0
        if not candidates:
            return None

        # For each candidate, count how many others (including itself) fall
        # within tolerance - the largest such cluster is the best guess at
        # "a real, repeatedly-seen position" this window supports. This
        # correctly handles multiple DIFFERENT false positives appearing in
        # the same window: none of them individually repeats enough to win.
        best_candidate, best_count = None, 0
        for c in candidates:
            count = sum(
                1 for d in candidates
                if abs(d[0] - c[0]) <= self.tolerance_px and abs(d[1] - c[1]) <= self.tolerance_px
            )
            if count > best_count:
                best_candidate, best_count = c, count
        self.last_hit_count = best_count

        if best_count < self.required_hits:
            return None

        # Confirmed - report the MOST RECENT candidate that belongs to the
        # winning cluster (freshest position estimate), not necessarily
        # best_candidate itself (which could be an older frame).
        latest = candidates[-1]
        if (abs(latest[0] - best_candidate[0]) <= self.tolerance_px
                and abs(latest[1] - best_candidate[1]) <= self.tolerance_px):
            return latest
        return best_candidate

    def reset(self):
        self.history.clear()
        self.last_hit_count = 0


class PID:
    def __init__(self, kp, ki, kd, deadband=0.0, out_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.deadband = deadband
        self.out_limit = out_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, error, now=None):
        if abs(error) < self.deadband:
            error = 0.0
        now = time.time() if now is None else now
        dt = max(1e-3, now - self.prev_time)
        self.integral = float(np.clip(self.integral + error * dt, -20.0, 20.0))
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.out_limit is not None:
            output = float(np.clip(output, -self.out_limit, self.out_limit))
        self.prev_error = error
        self.prev_time = now
        return output


def pixels_to_degrees(pixel_error, frame_dim, fov_deg):
    return (pixel_error / (frame_dim / 2.0)) * (fov_deg / 2.0)
