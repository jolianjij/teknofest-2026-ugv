"""Hybrid computer-vision detector for numbered UGV stage signs."""

from .config import ClassifierConfig, DetectorConfig, TemporalConfig
from .detector import BoundingBox, SignDetection, StageSignDetector
from .temporal import TemporalConfirmer

__all__ = [
    "BoundingBox",
    "ClassifierConfig",
    "DetectorConfig",
    "SignDetection",
    "StageSignDetector",
    "TemporalConfig",
    "TemporalConfirmer",
]

__version__ = "0.3.0"
