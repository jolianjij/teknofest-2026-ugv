from __future__ import annotations

from pathlib import Path

import pytest

from ugv_stage_sign_detector.config import DetectorConfig


def test_loads_shipped_configuration() -> None:
    path = Path(__file__).parents[1] / "config" / "detector.yaml"
    config = DetectorConfig.from_yaml(path)
    assert config.process_width == 960
    assert config.templates.labels == list(range(1, 13))
    assert config.classifier.mode == "template"
    assert config.temporal.required_hits == 4


def test_loads_ml_configuration_and_resolves_model_path() -> None:
    path = Path(__file__).parents[1] / "config" / "detector_ml.yaml"
    config = DetectorConfig.from_yaml(path)
    assert config.classifier.mode == "onnx"
    assert config.classifier.labels == list(range(0, 13))
    assert Path(config.classifier.model_path).is_absolute()
    assert Path(config.classifier.model_path).name == "stage_sign_classifier.onnx"


def test_rejects_unknown_keys(tmp_path: Path) -> None:
    config_file = tmp_path / "bad.yaml"
    config_file.write_text("detector:\n  red:\n    unknown: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Unknown RedMaskConfig settings"):
        DetectorConfig.from_yaml(config_file)


def test_onnx_mode_requires_model_path() -> None:
    config = DetectorConfig()
    config.classifier.mode = "onnx"
    with pytest.raises(ValueError, match="model_path is required"):
        config.validate()
