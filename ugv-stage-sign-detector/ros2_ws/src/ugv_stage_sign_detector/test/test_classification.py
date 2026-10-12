from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from ugv_stage_sign_detector.classification import (
    Classification,
    OnnxSignClassifier,
    preprocess_sign_patch,
)
from ugv_stage_sign_detector.config import DetectorConfig
from ugv_stage_sign_detector.detector import StageSignDetector
from ugv_stage_sign_detector.ml_dataset import class_name, parse_class_name
from ugv_stage_sign_detector.synthetic import make_scene


class StaticClassifier:
    def __init__(self, label: int) -> None:
        self.label = label

    def classify(self, patch_bgr, digit_mask=None) -> Classification:
        return Classification(
            label=self.label,
            confidence=0.99,
            runner_up=3,
            margin=0.90,
        )


def test_preprocessing_matches_model_tensor_contract() -> None:
    patch = np.full((160, 160, 3), 255, dtype=np.uint8)
    tensor = preprocess_sign_patch(patch, input_size=64)
    assert tensor.shape == (1, 1, 64, 64)
    assert tensor.dtype == np.float32
    assert np.allclose(tensor, 1.0)


def test_missing_onnx_model_fails_at_startup(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="does not exist"):
        OnnxSignClassifier(tmp_path / "missing.onnx", list(range(13)))


def test_detector_uses_injected_classifier_after_cv_candidate_detection() -> None:
    detector = StageSignDetector(DetectorConfig(), classifier=StaticClassifier(2))
    detections = detector.detect(make_scene(1))
    assert detections
    assert detections[0].sign_id == 2


def test_unknown_class_is_published_as_sign_zero_so_the_vehicle_can_stop() -> None:
    """An unreadable but geometry-valid sign must reach ROS, not be dropped.

    Silently discarding it means driving past an instruction we know is there.
    Sign 0 is the agreed signal for "a sign exists and we cannot read it"; the
    stage manager turns it into a stop without advancing the stage sequence.
    """
    detector = StageSignDetector(DetectorConfig(), classifier=StaticClassifier(0))
    detections = detector.detect(make_scene(1))
    assert detections
    assert detections[0].sign_id == 0
    # Confidence for an unknown is the geometry score alone - the classifier's
    # own confidence is meaningless precisely because it failed to classify.
    assert 0.0 < detections[0].confidence <= 1.0


def test_unknown_is_dropped_when_the_fail_safe_is_disabled() -> None:
    config = DetectorConfig()
    config.classifier.unknown_stops_vehicle = False
    detector = StageSignDetector(config, classifier=StaticClassifier(0))
    assert detector.detect(make_scene(1)) == []


def test_weak_geometry_unknown_is_dropped_rather_than_causing_nuisance_stops() -> None:
    """Only convincing red-ring geometry may halt the vehicle.

    Without this gate every red barrier fragment that survives candidate
    extraction would stop the run.
    """
    config = DetectorConfig()
    config.classifier.unknown_min_geometry = 0.999
    detector = StageSignDetector(config, classifier=StaticClassifier(0))
    assert detector.detect(make_scene(1)) == []


def test_known_signs_are_unaffected_by_the_unknown_fail_safe() -> None:
    detector = StageSignDetector(DetectorConfig(), classifier=StaticClassifier(2))
    detections = detector.detect(make_scene(1))
    assert [item.sign_id for item in detections] == [2]


def test_candidate_extraction_returns_rectified_training_patch() -> None:
    detector = StageSignDetector(DetectorConfig(), classifier=StaticClassifier(1))
    candidates = detector.extract_candidates(make_scene(1))
    assert candidates
    assert candidates[0].patch_bgr.shape == (160, 160, 3)
    assert candidates[0].digit_mask.shape == (96, 96)


@pytest.mark.parametrize(
    ("value", "expected"),
    [("unknown", 0), ("0", 0), ("1", 1), ("12", 12)],
)
def test_dataset_class_names(value: str, expected: int) -> None:
    assert parse_class_name(value) == expected
    assert class_name(expected) == ("unknown" if expected == 0 else str(expected))
