from __future__ import annotations

import cv2
import numpy as np
import pytest

from ugv_stage_sign_detector.config import DetectorConfig
from ugv_stage_sign_detector.detector import StageSignDetector
from ugv_stage_sign_detector.synthetic import make_scene


@pytest.fixture(scope="module")
def detector() -> StageSignDetector:
    return StageSignDetector(DetectorConfig())


@pytest.mark.parametrize("sign_id", range(1, 13))
def test_detects_every_clean_stage_sign(detector: StageSignDetector, sign_id: int) -> None:
    detections = detector.detect(make_scene(sign_id))
    assert detections
    assert detections[0].sign_id == sign_id
    assert detections[0].confidence >= 0.85


@pytest.mark.parametrize(
    ("sign_id", "angle", "horizontal_scale"),
    [
        (8, 8.0, 1.0),
        (8, -12.0, 1.0),
        (10, 6.0, 0.82),
        (12, -5.0, 0.70),
        (6, 0.0, 0.60),
        (3, 18.0, 0.85),
    ],
)
def test_detects_rotated_and_compressed_signs(
    detector: StageSignDetector,
    sign_id: int,
    angle: float,
    horizontal_scale: float,
) -> None:
    scene = make_scene(sign_id, angle_degrees=angle, horizontal_scale=horizontal_scale)
    detections = detector.detect(scene)
    assert detections
    assert detections[0].sign_id == sign_id
    assert detections[0].confidence >= 0.75


def test_red_white_barrier_is_not_a_sign(detector: StageSignDetector) -> None:
    frame = np.full((720, 1280, 3), (150, 175, 150), dtype=np.uint8)
    cv2.rectangle(frame, (0, 390), (1280, 455), (245, 245, 245), -1)
    for x in range(0, 1280, 220):
        cv2.rectangle(frame, (x, 390), (min(1279, x + 110), 455), (15, 15, 220), -1)
    assert detector.detect(frame) == []


@pytest.mark.parametrize("sign_id", [1, 2, 11, 12])
def test_distinguishes_one_two_pairs_under_blur(
    detector: StageSignDetector,
    sign_id: int,
) -> None:
    """Regression for real-camera confusion between 1/2 and 11/12."""
    angle = 7.0 if sign_id < 10 else -6.0
    scene = make_scene(sign_id, angle_degrees=angle, horizontal_scale=0.90)
    blurred = cv2.GaussianBlur(scene, (7, 7), 1.8)
    detections = detector.detect(blurred)
    assert detections
    assert detections[0].sign_id == sign_id
    assert detections[0].confidence >= 0.80
