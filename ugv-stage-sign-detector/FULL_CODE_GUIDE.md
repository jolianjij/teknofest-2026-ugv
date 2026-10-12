# Full Code Guide and Engineering Notes

This document explains the complete version 0.3.0 source. All operational code
is included in the repository and the safety-critical sections contain inline
comments. The recommended classifier is a locally trained compact CNN exported
to ONNX; there is no cloud or external OCR dependency.

## 1. End-to-end data flow

```text
Camera Module 3
    |
    | sensor_msgs/Image: /camera/image_raw
    v
StageSignDetectorNode
    |-- classical CV red-ring candidate extraction
    |-- ONNX crop classification: unknown or 1..12
    |-- frame observations: /perception/stage_sign/observation
    |-- debug image: /perception/stage_sign/debug_image
    |
    | temporally confirmed StageSign
    v
StageManagerNode
    |
    | order-validated StageTransition: /mission/stage_transition
    | current stage: /mission/current_stage
    v
StageActionDispatcherNode
    |
    | ExecuteStage goal: /mission/execute_stage
    v
Team behavior Action server
```

The emergency stop and communication-loss stop must remain independent of this
chain. This software is mission logic, not the competition-required independent
safety controller.

## 2. Source map

| File | Responsibility |
| --- | --- |
| `config.py` | Typed YAML configuration and validation |
| `detector.py` | Red-ring detection, structural checks, crop extraction and classifier orchestration |
| `classification.py` | Common result contract, exact preprocessing and OpenCV ONNX runtime |
| `templates.py` | Classical fallback reference generation and shape matching |
| `ml_dataset.py` | Augmented synthetic 13-class dataset generation |
| `capture_dataset.py` | Operator-labelled real candidate capture |
| `ml_model.py` | Laptop-only compact PyTorch CNN definition |
| `ml_train.py` | Train, test, export, OpenCV verification and ONNX evaluation |
| `temporal.py` | Four-of-six confirmation, locking, and rearming |
| `mission.py` | Strict course-order validation independent of ROS |
| `cli.py` | Windows/Linux webcam and video runner |
| `calibration.py` | Interactive HSV tuning tool |
| `synthetic.py` | Generated signs and scenes for deterministic tests |
| `ros_node.py` | `sensor_msgs/Image` to ROS sign observations/events |
| `stage_manager_node.py` | Confirmed sign to ordered behavior transition |
| `action_dispatcher_node.py` | Transition to generic ROS 2 Action goal |
| `mock_action_server.py` | Hardware-free Action integration test server |
| `ugv_stage_interfaces/msg` | `StageSign` and `StageTransition` messages |
| `ugv_stage_interfaces/action` | Generic `ExecuteStage` Action contract |

## 3. Hybrid vision algorithm

### 3.1 Resize and ROI

The Camera Module 3 can capture at 1280x720 while `process_width=960` limits CPU
work. Bounding boxes are converted back into original camera coordinates before
publication. The ROI is normalized from 0 to 1, so it remains usable if the
camera is standard or wide-angle.

### 3.2 Red segmentation

OpenCV represents hue from 0 to 179. Red appears at both ends of this interval,
so the code combines two masks:

```text
red range 1: H = 0..12
red range 2: H = 165..179
minimum saturation = 80
minimum value = 60
```

Morphological opening removes isolated red noise. Closing reconnects small gaps
in the ring caused by reflections, blur, and video compression.

### 3.3 Geometry rejection

For each external red contour, the detector checks:

- Minimum and maximum pixel area
- Width/height ratio
- Circularity
- Red fraction in the expected annulus
- White fraction in the expected centre
- Plausible black-digit foreground fraction

Circularity is calculated as:

```text
4 * pi * contour_area / perimeter^2
```

An ideal circle approaches 1.0. Long red barrier rectangles produce much lower
values and are rejected.

### 3.4 Candidate normalization

The accepted red contour is cropped with a small padding and resized to a fixed
square. This deliberately compensates for moderate horizontal compression when
the camera sees a circular sign at an angle.

### 3.5 Digit extraction

Only the inner circular region is considered. CLAHE improves local contrast in
partial shade, and Otsu thresholding selects a black/white threshold for each
candidate. Connected components below the configured area are removed as dirt
or compression noise.

### 3.6 ONNX classification (recommended)

The same preprocessing function is used during training and runtime: the
rectified BGR patch becomes grayscale, is resized to 64x64, converted to
`float32`, and normalized to approximately `[-1, 1]`. The CNN returns logits
for 13 classes: unknown and signs 1 through 12. OpenCV DNN calculates the
production inference result on Windows and the Raspberry Pi, so PyTorch is not
installed on the robot.

The candidate is accepted only when both the top-class confidence and the
difference from the runner-up pass their configured thresholds. Class zero is
always rejected. Only then does temporal confirmation see the result.

Training, dataset layout, real-session separation and deployment commands are
documented in `ML_TRAINING_GUIDE.md`.

### 3.7 Template classification (fallback)

The foreground is cropped and scaled to a 96x96 square while preserving aspect
ratio. Each label is compared using:

- Dice foreground overlap
- Symmetric chamfer distance between shape edges
- Separation between the best and second-best label

On Windows the code automatically uses:

```text
C:\Windows\Fonts\ariblk.ttf
```

This mode remains available through `detector.yaml` for calibration and
regression testing. It is not the recommended classifier for the competition
because live camera frames already demonstrated unstable 1/2 and 11/12
separation. `detector_ml.yaml` selects ONNX and does not silently fall back.

## 4. Temporal confirmation

The default policy is four matching detections in the latest six frames. A
single blurred or misclassified image therefore cannot change the mission.

After confirmation, the sign number is locked. The same physical sign cannot
produce another event until it has disappeared for eight frames. This is what
prevents repeated ROS Action goals while approaching or passing one sign.

Recommended field checks:

- Increase `required_hits` if false confirmations appear.
- Decrease it only if signs are visible for too few frames.
- Keep the final confidence threshold above the observed false-positive scores.
- Add odometry gating later if signs can reappear after a turn.

## 5. Course-order validation

`StageSequence` is independent of ROS and covered by unit tests. In strict mode,
after stage 2 only stage 3 is accepted. A false sign 8 detection cannot jump the
mission directly to the incline behavior.

The currently configured main-course sequence is 1 through 10. Signs 11 and 12
are recognized for the separate acceleration test, but should use their own
stage-manager configuration.

The mapping remains provisional because the supplied specification has a
conflict between Figure 1 and the Figure 2 example. Confirm it with the
competition committee before enabling autonomous actions.

## 6. ROS messages and Action

### StageSign

```text
std_msgs/Header header
uint8 sign_id
float32 confidence
sensor_msgs/RegionOfInterest bounding_box
bool confirmed
```

### StageTransition

```text
std_msgs/Header header
uint8 previous_stage
uint8 current_stage
string behavior
float32 confidence
```

### ExecuteStage

```text
# Goal
uint8 stage_id
string behavior
---
# Result
bool success
string message
---
# Feedback
float32 progress
string state
```

The dispatcher holds a transition if the Action server has not started. When a
new transition arrives while an older behavior is active, it requests
cancellation before dispatching the newest stage. The real Action server must
bring actuators to a safe state when processing a cancellation request.

## 7. Running the complete mock chain

Build the ROS workspace:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Terminal 1 - safe simulated Action server and dispatcher:

```bash
ros2 launch ugv_stage_sign_detector action_demo.launch.py
```

Terminal 2 - stage manager:

```bash
ros2 run ugv_stage_sign_detector stage_manager_node
```

Terminal 3 - simulate a confirmed sign 1:

```bash
ros2 topic pub --once /perception/stage_sign/confirmed \
  ugv_stage_interfaces/msg/StageSign \
  "{sign_id: 1, confidence: 0.95, confirmed: true}"
```

Expected result:

1. Stage manager accepts transition 0 to 1.
2. Dispatcher sends `fording_and_shower`.
3. Mock server reports progress and success.
4. No hardware commands are sent.

## 8. Windows development workflow

```powershell
cd ros2_ws\src\ugv_stage_sign_detector
py -3 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install -e .
ugv-generate-signs ..\..\..\test_output
ugv-sign-detect --source 0 --config config\detector.yaml
```

Use a phone or laptop webcam video before the Pi is available. Print generated
signs at smaller test sizes, record approaches from different angles, and test
that one physical sign produces exactly one JSON confirmation event.

## 9. Raspberry Pi notes

- Use a Raspberry Pi 5 active cooler to avoid CPU throttling.
- Use Ubuntu Server 24.04 64-bit with ROS 2 Jazzy unless the team has already
  standardized on another ROS 2 distribution.
- Camera Module 3 requires the Raspberry Pi 5-compatible 22-pin camera cable.
- Begin at 1280x720 capture and 960-pixel processing width.
- After testing, consider fixed focus/exposure/white balance to avoid hunting.
- Protect the lens from the shower stage with a hood and suitable clear cover.
- Record raw course video and ROS bags for offline threshold tuning.

## 10. What must be tested before vehicle use

The included automated tests validate software logic, not competition readiness.
Before using action outputs on the vehicle, test:

1. Standard versus wide Camera Module 3 field of view.
2. Minimum and maximum sign-recognition distance.
3. Direct sunlight, backlighting, shade, dusk, and reflections.
4. Motion blur at the maximum planned approach speed.
5. Camera vibration on aggregate, obstacle, and bumped-road sections.
6. Water droplets and partial occlusion.
7. Red/white barriers, cones, clothing, and logos as false-positive sources.
8. Every sign 1-12 from left/right angles and multiple distances.
9. ROS Action cancellation and safe actuator stopping.
10. Independent emergency stop and communication-loss shutdown.



for recording video and testing the signs deection 
.\.venv\Scripts\ugv-sign-detect.exe --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector_ml.yaml --output .\printed_sign_test.mp4