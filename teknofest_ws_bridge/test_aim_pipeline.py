"""
Regression tests for the laser-aim pipeline's noise-robustness (2026-09-10).

Both target_detection.py and aim_tracker.py's AcquisitionConfirmer are
deliberately ROS-independent so they can be tested exactly like this,
without a robot, a camera, or the Orange Pi - same rationale already
documented in target_detection.py's own docstring.

These exist because today's changes (loosening detection thresholds for
10m range, then adding an aspect-ratio filter and a multi-frame
acquisition gate to claw back the resulting false-positive risk) had zero
prior test coverage to check against - run via: pytest test_aim_pipeline.py
"""
from collections import deque

import numpy as np
import pytest
import cv2

from target_detection import detect_target, AcquisitionConfirmer


# ---------- target_detection.py: synthetic detect-rate + noise rejection ----------

def _draw_target(canvas_size, outer_radius_px, cx=None, cy=None, noise_level=8, seed=0):
    """1:2:3 black/white/black ring target, matching the real spec geometry."""
    rng = np.random.RandomState(seed)
    img = np.full((canvas_size, canvas_size, 3), 200, dtype=np.uint8)
    if noise_level:
        noise = rng.randint(-noise_level, noise_level + 1, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    cx = cx if cx is not None else canvas_size // 2
    cy = cy if cy is not None else canvas_size // 2
    r_outer = outer_radius_px
    r_mid = outer_radius_px * (2.0 / 3.0)
    r_inner = outer_radius_px * (1.0 / 3.0)
    cv2.circle(img, (cx, cy), int(r_outer), (0, 0, 0), -1)
    cv2.circle(img, (cx, cy), int(r_mid), (255, 255, 255), -1)
    cv2.circle(img, (cx, cy), int(r_inner), (0, 0, 0), -1)
    return img, (cx, cy)


def _draw_noise_scene(canvas_size, n_shapes, seed):
    """Assorted non-target shapes (solid circles, rects, ellipses) - none of
    these are a 1:2:3 concentric ring, so none should ever be detected."""
    rng = np.random.RandomState(seed)
    img = np.full((canvas_size, canvas_size, 3), 200, dtype=np.uint8)
    for _ in range(n_shapes):
        x, y = rng.randint(50, canvas_size - 50, size=2)
        size = rng.randint(15, 80)
        shape_type = rng.choice(['circle', 'rect', 'ellipse'])
        color = tuple(int(v) for v in rng.randint(0, 256, size=3))
        if shape_type == 'circle':
            cv2.circle(img, (x, y), size, color, -1)
        elif shape_type == 'rect':
            cv2.rectangle(img, (x - size, y - size // 2), (x + size, y + size // 2), color, -1)
        else:
            cv2.ellipse(img, (x, y), (size, size // 2), int(rng.randint(0, 180)), 0, 360, color, -1)
    return img


@pytest.mark.parametrize('trial', range(10))
def test_close_range_target_still_detected(trial):
    """Regression check: today's 10m-range threshold loosening must not have
    broken detection of a large, close-range target."""
    img, (cx, cy) = _draw_target(640, outer_radius_px=140, seed=trial)
    det = detect_target(img)
    assert det is not None, 'close-range target should always be detected'
    dx, dy, r, conf = det
    assert abs(dx - cx) < 3 and abs(dy - cy) < 3


@pytest.mark.parametrize('trial', range(10))
def test_10m_equivalent_target_detected(trial):
    """The actual improvement being validated: ~85px-across target (42px
    radius), matching the aim camera's real geometry at 10m per
    camera_manager_node.py's PROFILES comment."""
    img, (cx, cy) = _draw_target(640, outer_radius_px=42, seed=trial)
    det = detect_target(img)
    assert det is not None, '10m-equivalent target should be detected after the range fix'
    dx, dy, r, conf = det
    assert abs(dx - cx) < 5 and abs(dy - cy) < 5


@pytest.mark.parametrize('seed', range(15))
def test_random_noise_scenes_never_detected(seed):
    """Noise-rejection check: assorted non-ring shapes must never be
    mistaken for the target, even with the loosened 10m thresholds."""
    img = _draw_noise_scene(640, n_shapes=10, seed=seed)
    det = detect_target(img)
    assert det is None, f'false positive on synthetic noise scene (seed={seed})'


# ---------- aim_tracker.py: AcquisitionConfirmer spatial-clustering logic ----------

def test_confirms_a_real_jittery_but_consistent_target():
    c = AcquisitionConfirmer()
    detections = [(500, 400, 40), (503, 398, 41), None, (498, 402, 39), (501, 399, 40), (502, 401, 40)]
    results = [c.update(d) for d in detections]
    assert results[-1] is not None, 'should confirm once enough clustered hits land in the window'


def test_rejects_scattered_random_noise():
    c = AcquisitionConfirmer()
    detections = [(100, 100, 20), (900, 800, 30), (300, 600, 15), (700, 200, 25), (50, 900, 10), (600, 50, 35)]
    results = [c.update(d) for d in detections]
    assert all(r is None for r in results), 'scattered noise that never clusters must never confirm'


def test_rejects_two_competing_clusters_each_below_threshold():
    """Two DIFFERENT false positives, each appearing 3 times (below the
    4-hit requirement) - neither should individually reach confirmation,
    which a naive 'any repeat counts' check could get wrong."""
    c = AcquisitionConfirmer()
    detections = [(100, 100, 20), (900, 800, 30), (105, 102, 20), (895, 805, 30), (98, 99, 20), (902, 798, 30)]
    results = [c.update(d) for d in detections]
    assert all(r is None for r in results)


def test_confirms_sparse_real_target_with_gaps():
    c = AcquisitionConfirmer()
    detections = [(500, 400, 40), None, (502, 399, 40), None, (499, 401, 40), (501, 400, 40)]
    results = [c.update(d) for d in detections]
    assert results[-1] is not None


def test_reset_clears_history():
    c = AcquisitionConfirmer()
    for d in [(500, 400, 40)] * 4:
        c.update(d)
    c.reset()
    assert len(c.history) == 0
    assert c.last_hit_count == 0
