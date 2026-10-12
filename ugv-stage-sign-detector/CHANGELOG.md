# Changelog

## 0.3.0 - Hybrid classifier foundation

- Kept HSV red-ring detection, geometry validation, temporal confirmation,
  stage sequencing, and ROS 2 Action boundaries.
- Added a compact 13-class (`unknown`, `1`-`12`) CNN training definition.
- Added OpenCV DNN ONNX inference with shared training/runtime preprocessing.
- Added confidence and top-two margin rejection; unknown can never reach ROS.
- Added augmented synthetic dataset generation and operator-labelled real crop
  capture independent of the old template prediction.
- Added training, held-out evaluation, confusion matrix, PyTorch checkpoint,
  ONNX export, and OpenCV export verification tools.
- Added `detector_ml.yaml`, ROS/CLI classifier parameters, and Pi ML launch.
- Preserved `detector.yaml` as the classical regression/calibration fallback.
- Added regression tests for preprocessing, classifier injection, unknown
  rejection, model-path validation, and training-candidate extraction.
