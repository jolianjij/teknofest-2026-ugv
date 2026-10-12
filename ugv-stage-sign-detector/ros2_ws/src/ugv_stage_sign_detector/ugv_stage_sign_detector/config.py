"""Typed configuration for the detector.

Keeping thresholds outside the image-processing code is important for this
project. Outdoor illumination, camera field of view, mounting height, and sign
distance are not final yet, so every field-sensitive value can be changed in
``config/detector.yaml`` without editing Python.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any, Iterable

import yaml


@dataclass(slots=True)
class RoiConfig:
    """Normalized image region searched for signs.

    Values are fractions rather than pixels, which makes one configuration
    usable at several camera resolutions. For example, ``y_max=0.9`` ignores
    the lowest ten percent of the image, where the vehicle body may appear.
    """

    x_min: float = 0.0
    y_min: float = 0.0
    x_max: float = 1.0
    y_max: float = 0.9


@dataclass(slots=True)
class RedMaskConfig:
    """HSV thresholds used to isolate the red circular border.

    Red crosses the 0/179 boundary in OpenCV's HSV representation. Therefore,
    it must be represented by two hue intervals instead of one.
    """
    low_hue_1: int = 0
    high_hue_1: int = 12
    low_hue_2: int = 165
    high_hue_2: int = 179
    min_saturation: int = 80
    min_value: int = 60
    morphology_kernel: int = 5


@dataclass(slots=True)
class GeometryConfig:
    """Shape and colour checks that reject barriers and random red objects."""
    min_area_px: int = 180
    max_area_fraction: float = 0.20
    min_circularity: float = 0.55
    min_aspect_ratio: float = 0.55
    max_aspect_ratio: float = 1.55
    min_red_annulus_fraction: float = 0.28
    min_white_inner_fraction: float = 0.42
    annulus_inner_radius: float = 0.34
    annulus_outer_radius: float = 0.50
    digit_region_radius: float = 0.34
    min_digit_fraction: float = 0.008
    max_digit_fraction: float = 0.34


@dataclass(slots=True)
class TemplateConfig:
    """Settings for classical, font-template digit classification."""
    labels: list[int] = field(default_factory=lambda: list(range(1, 13)))
    rectified_size: int = 160
    normalized_digit_size: int = 96
    normalized_margin: int = 8
    font_path: str = ""
    min_match_confidence: float = 0.58
    min_classification_margin: float = 0.015


@dataclass(slots=True)
class ClassifierConfig:
    """Select and configure the per-candidate number classifier.

    ``template`` preserves the original no-training-data implementation.
    ``onnx`` runs the small CNN exported by ``ugv-ml-train``.  Label zero is a
    deliberate unknown/reject class.

    ``unknown_stops_vehicle`` makes label zero a safety event rather than a
    silent drop: a geometry-valid sign the model cannot name is published as
    sign 0 so the mission layer can halt instead of driving past an unread
    instruction.  ``unknown_min_geometry`` is the only gate applied to those
    candidates, because the classifier confidence and margin gates are exactly
    what an unknown has already failed.  Raise it if red barriers or other
    round red objects start producing nuisance stops.
    """

    mode: str = "template"
    model_path: str = ""
    labels: list[int] = field(default_factory=lambda: list(range(0, 13)))
    input_size: int = 64
    min_confidence: float = 0.85
    min_margin: float = 0.20
    unknown_stops_vehicle: bool = True
    unknown_min_geometry: float = 0.62


@dataclass(slots=True)
class TemporalConfig:
    """Settings for repeated-frame confirmation and duplicate suppression."""
    history_size: int = 6
    required_hits: int = 4
    # Prefer no mission transition over locking a weak classification. Real
    # field recordings can be used to tune this after precision is measured.
    min_confidence: float = 0.78
    disappear_frames_to_rearm: int = 8


@dataclass(slots=True)
class DetectorConfig:
    """Root configuration object shared by the CLI and ROS node."""
    process_width: int = 960
    roi: RoiConfig = field(default_factory=RoiConfig)
    red: RedMaskConfig = field(default_factory=RedMaskConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    templates: TemplateConfig = field(default_factory=TemplateConfig)
    classifier: ClassifierConfig = field(default_factory=ClassifierConfig)
    temporal: TemporalConfig = field(default_factory=TemporalConfig)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "DetectorConfig":
        with Path(path).open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
        raw = raw.get("detector", raw)
        classifier = _construct(ClassifierConfig, raw.get("classifier", {}))
        if classifier.model_path:
            model_path = Path(classifier.model_path).expanduser()
            if not model_path.is_absolute():
                model_path = Path(path).resolve().parent / model_path
            classifier.model_path = str(model_path.resolve())
        config = cls(
            process_width=int(raw.get("process_width", cls().process_width)),
            roi=_construct(RoiConfig, raw.get("roi", {})),
            red=_construct(RedMaskConfig, raw.get("red", {})),
            geometry=_construct(GeometryConfig, raw.get("geometry", {})),
            templates=_construct(TemplateConfig, raw.get("templates", {})),
            classifier=classifier,
            temporal=_construct(TemporalConfig, raw.get("temporal", {})),
        )
        config.validate()
        return config

    def validate(self) -> None:
        """Fail early when a configuration could produce undefined behaviour."""
        if self.process_width < 160:
            raise ValueError("process_width must be at least 160 pixels")
        _validate_fraction_range(
            (self.roi.x_min, self.roi.y_min, self.roi.x_max, self.roi.y_max),
            "ROI values",
        )
        if self.roi.x_min >= self.roi.x_max or self.roi.y_min >= self.roi.y_max:
            raise ValueError("ROI minimums must be lower than maximums")
        if self.red.morphology_kernel < 1 or self.red.morphology_kernel % 2 == 0:
            raise ValueError("morphology_kernel must be a positive odd integer")
        if self.templates.rectified_size < 64:
            raise ValueError("rectified_size must be at least 64")
        if not self.templates.labels:
            raise ValueError("At least one template label is required")
        if len(set(self.templates.labels)) != len(self.templates.labels):
            raise ValueError("Template labels must be unique")
        if self.classifier.mode not in {"template", "onnx"}:
            raise ValueError("classifier.mode must be either 'template' or 'onnx'")
        if self.classifier.mode == "onnx" and not self.classifier.model_path:
            raise ValueError("classifier.model_path is required when classifier.mode is 'onnx'")
        if self.classifier.input_size < 32:
            raise ValueError("classifier.input_size must be at least 32")
        if not self.classifier.labels:
            raise ValueError("classifier.labels cannot be empty")
        if len(set(self.classifier.labels)) != len(self.classifier.labels):
            raise ValueError("classifier.labels must be unique")
        if any(label < 0 or label > 255 for label in self.classifier.labels):
            raise ValueError("classifier.labels must fit in an unsigned byte")
        _validate_fraction_range(
            (
                self.classifier.min_confidence,
                self.classifier.min_margin,
                self.classifier.unknown_min_geometry,
            ),
            "Classifier thresholds",
        )
        if self.temporal.required_hits > self.temporal.history_size:
            raise ValueError("required_hits cannot exceed history_size")


def _construct(model: type[Any], values: dict[str, Any]) -> Any:
    known = {item.name for item in fields(model)}
    unexpected = set(values) - known
    if unexpected:
        raise ValueError(f"Unknown {model.__name__} settings: {sorted(unexpected)}")
    return model(**values)


def _validate_fraction_range(values: Iterable[float], name: str) -> None:
    if any(value < 0.0 or value > 1.0 for value in values):
        raise ValueError(f"{name} must be between 0 and 1")
