"""Shared interfaces for template and neural-network sign classifiers.

The red-ring detector owns candidate detection and geometric validation.  A
classifier receives only the rectified candidate, which keeps the ML problem
small and allows the rest of the ROS pipeline to remain unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import cv2
import numpy as np


UNKNOWN_LABEL = 0


@dataclass(frozen=True, slots=True)
class Classification:
    """One classifier result before temporal confirmation."""

    label: int
    confidence: float
    runner_up: int
    margin: float


class SignClassifier(Protocol):
    """Interface implemented by both the classical and ONNX classifiers."""

    def classify(
        self,
        patch_bgr: np.ndarray,
        digit_mask: np.ndarray | None = None,
    ) -> Classification:
        """Classify one rectified BGR sign candidate."""


def preprocess_sign_patch(patch_bgr: np.ndarray, input_size: int = 64) -> np.ndarray:
    """Convert a rectified BGR sign patch to the model's ``1x1xHxW`` input.

    Training and runtime call this exact function, preventing subtle differences
    in colour conversion, scaling, or normalization between the laptop and Pi.
    Pixel values are mapped from ``[0, 255]`` to approximately ``[-1, 1]``.
    """

    if patch_bgr is None or patch_bgr.size == 0:
        raise ValueError("Cannot preprocess an empty sign patch")
    if input_size < 32:
        raise ValueError("ML input_size must be at least 32 pixels")

    if patch_bgr.ndim == 2:
        gray = patch_bgr
    elif patch_bgr.ndim == 3 and patch_bgr.shape[2] == 3:
        gray = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError("Expected a grayscale or three-channel BGR sign patch")

    resized = cv2.resize(gray, (input_size, input_size), interpolation=cv2.INTER_AREA)
    normalized = resized.astype(np.float32) / 127.5 - 1.0
    return np.ascontiguousarray(normalized[None, None, :, :], dtype=np.float32)


class OnnxSignClassifier:
    """Small 1-12 sign classifier executed by OpenCV DNN on CPU.

    No PyTorch installation is required at runtime.  PyTorch is used only by
    the separate training command to produce the ONNX file.
    """

    def __init__(
        self,
        model_path: str | Path,
        labels: list[int] | tuple[int, ...],
        input_size: int = 64,
    ) -> None:
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"ONNX sign-classifier model does not exist: {path.resolve()}")
        if len(labels) < 2 or len(set(labels)) != len(labels):
            raise ValueError("ONNX classifier labels must contain at least two unique values")

        self.model_path = path.resolve()
        self.labels = tuple(int(label) for label in labels)
        self.input_size = int(input_size)
        self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def classify(
        self,
        patch_bgr: np.ndarray,
        digit_mask: np.ndarray | None = None,
    ) -> Classification:
        """Run the model and return softmax confidence and top-two margin."""

        del digit_mask  # Kept in the common interface for the template adapter.
        tensor = preprocess_sign_patch(patch_bgr, self.input_size)
        self.net.setInput(tensor)
        logits = np.asarray(self.net.forward(), dtype=np.float32).reshape(-1)
        if logits.size != len(self.labels):
            raise RuntimeError(
                f"ONNX model returned {logits.size} classes, but configuration defines "
                f"{len(self.labels)} labels"
            )
        if not np.all(np.isfinite(logits)):
            raise RuntimeError("ONNX model returned non-finite classification values")

        probabilities = _softmax(logits)
        ordered = np.argsort(probabilities)[::-1]
        best_index = int(ordered[0])
        second_index = int(ordered[1])
        confidence = float(probabilities[best_index])
        runner_up_confidence = float(probabilities[second_index])
        return Classification(
            label=self.labels[best_index],
            confidence=confidence,
            runner_up=self.labels[second_index],
            margin=max(0.0, confidence - runner_up_confidence),
        )


def _softmax(values: np.ndarray) -> np.ndarray:
    shifted = values - float(np.max(values))
    exponentials = np.exp(shifted)
    denominator = float(np.sum(exponentials))
    if denominator <= 0.0:
        raise RuntimeError("Could not normalize ONNX classifier output")
    return exponentials / denominator
