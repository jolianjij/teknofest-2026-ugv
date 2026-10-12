from __future__ import annotations

import pytest

from ugv_stage_sign_detector.config import DetectorConfig
from ugv_stage_sign_detector.templates import TemplateRepository


def test_explicit_template_directory_must_exist(tmp_path) -> None:
    missing = tmp_path / "missing_templates"
    with pytest.raises(FileNotFoundError, match="does not exist"):
        TemplateRepository.load_or_generate(DetectorConfig().templates, missing)


def test_explicit_template_directory_must_be_complete(tmp_path) -> None:
    incomplete = tmp_path / "incomplete_templates"
    incomplete.mkdir()
    with pytest.raises(FileNotFoundError, match="incomplete"):
        TemplateRepository.load_or_generate(DetectorConfig().templates, incomplete)
