"""Loads sign_cnn.onnx (see train.py) and classifies a cropped BGR ROI into
one of 11 classes (0=not a sign, 1-10=stage number). Shared by
sign_detector.py (deployment) and any local testing script.

Deployment note: torch 2.10's ONNX exporter externalizes weights into a
sibling `sign_cnn.onnx.data` file - both `sign_cnn.onnx` and
`sign_cnn.onnx.data` must be copied to the Orange Pi together (same
directory), or onnxruntime will fail to load the model."""
from pathlib import Path

import cv2
import numpy as np

MODEL_PATH = Path(__file__).parent / "sign_cnn.onnx"
IMG_SIZE = 64
MEAN = np.array([0.5, 0.5, 0.5], dtype=np.float32)
STD = np.array([0.25, 0.25, 0.25], dtype=np.float32)


class SignClassifier:
    def __init__(self, model_path=MODEL_PATH):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])

    def classify(self, roi_bgr):
        """Returns (predicted_class 0-10, confidence 0-1)."""
        if roi_bgr is None or roi_bgr.size == 0:
            return 0, 0.0
        rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (IMG_SIZE, IMG_SIZE), interpolation=cv2.INTER_LINEAR)
        arr = (resized.astype(np.float32) / 255.0 - MEAN) / STD
        arr = np.transpose(arr, (2, 0, 1))[None, ...]

        logits = self.session.run(None, {"input": arr})[0][0]
        probs = np.exp(logits - logits.max())
        probs /= probs.sum()
        cls = int(probs.argmax())
        return cls, float(probs[cls])
