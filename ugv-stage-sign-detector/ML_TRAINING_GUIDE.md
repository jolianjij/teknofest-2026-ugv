# Hybrid ML Training Guide

This version keeps the existing red-ring/geometry detector and replaces only
the fragile template-number decision with a small 13-class CNN:

```text
camera -> red ring -> geometry checks -> rectified crop -> CNN -> temporal confirmation -> ROS 2
```

The classes are `unknown`, `1`, `2`, ..., `12`. The unknown class is a safety
reject class and is never published as a stage transition.

## 1. Windows 11 training setup (Python 3.10)

Run these commands from the extracted project root:

```powershell
cd C:\UGV\ugv-stage-sign-detector
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
.\.venv\Scripts\python.exe -m pip install -r .\requirements-ml.txt
.\.venv\Scripts\python.exe -m pip install -e .\ros2_ws\src\ugv_stage_sign_detector
```

PyTorch and ONNX are laptop-side training dependencies. The Raspberry Pi needs
only OpenCV because it runs the exported `.onnx` model through OpenCV DNN.

## 2. Generate the synthetic bootstrap dataset

```powershell
.\.venv\Scripts\ugv-ml-generate-dataset.exe .\datasets\synthetic --font "C:\Windows\Fonts\ariblk.ttf"
```

The defaults create 1,200 training, 200 validation, and 200 test images per
class. Generation adds perspective, rotation, blur, brightness, noise, glare,
JPEG artifacts, horizontal compression, and screen-like lines.

Expected hierarchy:

```text
datasets/
└── synthetic/
    ├── train/
    │   ├── unknown/
    │   ├── 1/
    │   ├── 2/
    │   └── ... 12/
    ├── val/
    │   └── unknown, 1 ... 12
    ├── test/
    │   └── unknown, 1 ... 12
    └── manifest.csv
```

## 3. Capture real camera samples

Synthetic data is only a starting point. Real crops are required to learn the
webcam/Camera Module 3 exposure, glare, printer, phone screen, distance, and
motion characteristics.

Capture one label and one recording session at a time. Press Space to save a
good crop and Q to quit:

```powershell
.\.venv\Scripts\ugv-ml-capture.exe .\datasets\real --label 1 --split train --session laptop_room_front --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector.yaml
```

Repeat for `2`, `11`, `12`, and then every other sign. Capture the unknown class
by showing red-ring-like objects or invalid numbered signs:

```powershell
.\.venv\Scripts\ugv-ml-capture.exe .\datasets\real --label unknown --split train --session hard_negatives_room --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector.yaml
```

Recommended first target:

- 200-500 independently selected real crops per class for training.
- At least 100 per class in separate validation/test sessions.
- Extra coverage for `1`, `2`, `11`, and `12`.
- Different distances, angles, light levels, blur, glare, printed signs, and
  phone-displayed signs.

Do not put adjacent frames from one video into different splits. For example,
capture `laptop_room_front` only into train, `laptop_window_angle` only into
validation, and `laptop_outdoor_motion` only into test. Otherwise the reported
accuracy will be unrealistically high.

Automatic capture is available but produces correlated frames; use it only for
quick collection and keep its entire session in one split:

```powershell
.\.venv\Scripts\ugv-ml-capture.exe .\datasets\real --label 1 --split train --session auto_room_1 --source 0 --auto --interval 15 --max-images 300
```

## 4. Train and export ONNX

The output location below matches `config\detector_ml.yaml`:

```powershell
.\.venv\Scripts\ugv-ml-train.exe .\datasets\synthetic --additional-root .\datasets\real --output .\ros2_ws\src\ugv_stage_sign_detector\models\stage_sign_classifier.onnx --epochs 25 --batch-size 64
```

Training produces:

- `stage_sign_classifier.onnx`: Raspberry Pi/OpenCV runtime model.
- `stage_sign_classifier.pt`: best PyTorch checkpoint for future retraining.
- `stage_sign_classifier_history.csv`: per-epoch loss and accuracy.
- `stage_sign_classifier_metrics.json`: held-out test accuracy, per-class
  accuracy, and the confusion matrix.

The command verifies that OpenCV can load and execute the exported ONNX file.

## 5. Evaluate again after adding data

```powershell
.\.venv\Scripts\ugv-ml-evaluate.exe .\ros2_ws\src\ugv_stage_sign_detector\models\stage_sign_classifier.onnx .\datasets\real --split test --output .\real_test_metrics.json
```

Inspect the confusion matrix, particularly the `1/2` and `11/12` cells. Do not
enable autonomous actions merely because overall accuracy is high; the false
stage-trigger rate and every individual class must be acceptable.

## 6. Test the hybrid detector on Windows

```powershell
.\.venv\Scripts\ugv-sign-detect.exe --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector_ml.yaml
```

The ONNX configuration starts only when the model exists. It intentionally
does not silently fall back to template matching.

Provisional acceptance thresholds are in `detector_ml.yaml`:

```yaml
classifier:
  min_confidence: 0.85
  min_margin: 0.20
```

Tune them using separate real validation sessions. A rejected/unknown result is
safer than an incorrect stage transition. Temporal confirmation still requires
four matching observations within six frames.

## 7. Raspberry Pi / ROS 2 deployment

Copy the complete project, including the trained ONNX file, to the Pi and build:

```bash
cd ~/ugv-stage-sign-detector/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The Pi launch file selects `detector_ml.yaml`:

```bash
ros2 launch ugv_stage_sign_detector pi_camera_and_detector.launch.py
```

For a custom camera topic:

```bash
ros2 launch ugv_stage_sign_detector sign_detector.launch.py \
  config_file:=$(ros2 pkg prefix ugv_stage_sign_detector)/share/ugv_stage_sign_detector/config/detector_ml.yaml \
  image_topic:=/camera/image_raw
```

Before enabling the real action server, test with recorded course video and the
included mock action server. Keep the hardware emergency stop independent of
this perception process.
