# UGV Stage Sign Detector v0.3.0

Hybrid computer vision for the numbered stage signs in the TEKNOFEST 2026
Unmanned Ground Vehicle competition. Classical CV locates and validates the
red-ring sign; a small ONNX CNN classifies the rectified crop. The project runs
as a normal OpenCV application on Windows and as ROS 2 nodes on a Raspberry Pi 5.

On a new computer, begin with [SETUP_FROM_ZERO.md](SETUP_FROM_ZERO.md) for the
exact folder hierarchy and step-by-step Windows and Raspberry Pi installation.
Then use [FULL_CODE_GUIDE.md](FULL_CODE_GUIDE.md) for a file-by-file explanation
of the fully commented source and complete perception-to-ROS-Action data flow.
Use [ML_TRAINING_GUIDE.md](ML_TRAINING_GUIDE.md) to generate data, capture real
camera failures, train the CNN, export ONNX, evaluate it, and deploy it to ROS 2.

> Status: the hybrid code and classical fallback are implemented. A trained
> ONNX model is intentionally not claimed as competition-ready until real
> laptop-camera and Camera Module 3 sessions are captured and held-out tests
> pass. Field calibration remains mandatory before controlling a vehicle.

## Design

The recommended runtime is hybrid:

1. Segment the two red hue ranges in HSV.
2. Find circular red candidates.
3. Require a red annulus, white center, and black foreground.
4. Rectify the sign to a stable square crop.
5. Classify `unknown` or signs 1-12 with a compact ONNX CNN.
6. Confirm a sign only after repeated detections across several frames.

The original template classifier remains available through `detector.yaml` for
regression testing. The real hybrid configuration is `detector_ml.yaml`; it
fails clearly if its trained model is missing instead of silently reverting to
the unreliable template result.

The vision node never sends motor commands. The stage manager validates sign
order and publishes a behavior transition. A separate action dispatcher turns
that transition into a generic ROS 2 `ExecuteStage` goal for the vehicle-control
teammate's action server.

## Repository layout

```text
ros2_ws/src/
  ugv_stage_interfaces/       ROS messages and ExecuteStage action
  ugv_stage_sign_detector/    OpenCV core, CLI, ROS nodes, config and tests
requirements.txt
requirements-ml.txt            Laptop-only PyTorch/ONNX training dependencies
ML_TRAINING_GUIDE.md            Dataset, training, evaluation and deployment
test_output/                  Generated test artifacts (after running tools)
```

## Windows 11 quick start

Python 3.10 is the documented Windows development version.

```powershell
cd ros2_ws\src\ugv_stage_sign_detector
py -3.10 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e .
```

Generate signs, digit templates, and test scenes. On most Windows systems,
Arial Black is found automatically at `C:\Windows\Fonts\ariblk.ttf`.

```powershell
ugv-generate-signs ..\..\..\test_output
```

Test with the laptop webcam:

```powershell
ugv-sign-detect --source 0 --config config\detector.yaml
```

That command exercises the classical fallback. After following the ML guide
and creating the ONNX model, run the hybrid detector with:

```powershell
ugv-sign-detect --source 0 --config config\detector_ml.yaml
```

Test a recorded video and save an annotated result:

```powershell
ugv-sign-detect `
  --source C:\videos\course.mp4 `
  --config config\detector.yaml `
  --templates ..\..\..\test_output\digit_templates `
  --output C:\videos\course_detected.mp4
```

Press `Q` or Escape to exit.

## Camera calibration tool

Lighting changes the apparent red hue. Use the interactive HSV tool with a
webcam or recorded test run:

```powershell
ugv-sign-calibrate `
  --source 0 `
  --config config\detector.yaml `
  --save calibrated_detector.yaml
```

Adjust the sliders until the red mask contains the circular sign border but as
little of the barriers and environment as possible. Press `S` to save.

## Raspberry Pi target

Recommended conservative baseline:

- Raspberry Pi 5, 8 GB, with active cooling
- Ubuntu Server 24.04 64-bit
- ROS 2 Jazzy
- Camera Module 3 through libcamera and `camera_ros`
- Capture at 1280x720; process at 960 pixels wide initially

Use the Raspberry Pi 5-compatible 22-pin-to-15-pin camera cable. First confirm
that the operating system can see the camera, then confirm that `camera_ros`
publishes `/camera/image_raw` before starting this detector.

Create a workspace and copy this repository's two packages into `src` (the
provided `ros2_ws` already has this layout):

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start the camera, detector, and stage manager together after installing
`camera_ros`:

```bash
ros2 launch ugv_stage_sign_detector pi_camera_and_detector.launch.py
```

Or start only the detector and stage manager for any camera topic:

```bash
ros2 launch ugv_stage_sign_detector sign_detector.launch.py \
  image_topic:=/camera/image_raw
```

Inspect the output:

```bash
ros2 topic echo /perception/stage_sign/confirmed
ros2 topic echo /mission/stage_transition
ros2 topic echo /mission/current_stage
```

The debug image is published on `/perception/stage_sign/debug_image`.

To test the Action layer without hardware, launch the safe mock server:

```bash
ros2 launch ugv_stage_sign_detector action_demo.launch.py
```

## ROS 2 integration contract

### `ugv_stage_interfaces/msg/StageSign`

```text
std_msgs/Header header
uint8 sign_id
float32 confidence
sensor_msgs/RegionOfInterest bounding_box
bool confirmed
```

Topics:

| Topic | Meaning | QoS |
| --- | --- | --- |
| `/perception/stage_sign/observation` | Every accepted frame observation | Reliable |
| `/perception/stage_sign/confirmed` | Temporally confirmed, one-shot sign event | Reliable |
| `/perception/stage_sign/debug_image` | Annotated image for operators | Sensor data |
| `/mission/stage_transition` | Accepted sequential stage and behavior name | Reliable |
| `/mission/current_stage` | Current stage; available to late subscribers | Transient local |

### `ugv_stage_interfaces/action/ExecuteStage`

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

The dispatcher sends goals to `/mission/execute_stage`. When the next valid
stage arrives, it cancels any previous active behavior before sending the new
goal. `mock_stage_action_server` simulates this contract without touching GPIO,
motors, brakes, steering, or the laser.

The control teammate can implement the generic `/mission/execute_stage` server,
subscribe directly to `/mission/stage_transition`, or replace only the included
dispatcher with their own Action/behavior-tree adapter. Keeping this boundary
separate prevents perception code from directly actuating the vehicle.

## Provisional main-course mapping

The mapping below is inferred from Figure 1 of the July 24 competition
specification. Figure 2 contains a conflicting example, so the committee's
official mapping must be confirmed before final competition use.

| Sign | Behavior string | Stage |
| ---: | --- | --- |
| 1 | `fording_and_shower` | Water and shower |
| 2 | `aggregate_road` | Aggregate/rocky road |
| 3 | `side_slope` | Side slope |
| 4 | `vertical_obstacle` | Vertical obstacle |
| 5 | `traffic_cones` | Traffic cones |
| 6 | `sliding_doors` | Sliding doors |
| 7 | `bumped_road` | Bumped road |
| 8 | `incline_stop` | Incline and two-second stop |
| 9 | `fire_zone` | Target detection and laser designation |
| 10 | `decline_stop` | Decline and two-second stop |

Acceleration signs 11 and 12 are generated and recognized, but the included
main-course stage manager uses sequence 1-10. Configure a separate sequence
`[11, 12]` for the acceleration run.

## Configuration

All thresholds are in `config/detector.yaml`:

- ROI is normalized, so it works with standard or wide Camera Module 3.
- Both red hue ranges, saturation, and brightness are configurable.
- Geometry thresholds reject the red/white course barriers.
- Template, ONNX confidence/margin, and temporal thresholds are independent.
- Confirmation defaults to four hits in the latest six frames.

For the real course, collect video in bright sun, shade, wet-lens conditions,
turns, and vibration. Tune on one recording and validate on separate recordings
to avoid overfitting thresholds to a single test.

## Tests

From `ros2_ws/src/ugv_stage_sign_detector`:

```bash
python -m pytest test -q
```

The tests cover all 12 labels, moderate image rotation, horizontal compression,
red/white barrier rejection, configuration validation, confirmation, duplicate
suppression, and rearming.

## Safety boundaries

- Do not use the Raspberry Pi/ROS process as the vehicle's only emergency stop.
- Keep communication-loss and emergency-stop hardware/software independent of
  the main control computer, as required by the competition specification.
- Require odometry/IMU confirmation for the two-second stationary stages.
- Interlock the laser so vehicle and aiming mechanism motion are impossible
  while it is active.
- Confirm the stage-number mapping with the competition committee before
  enabling autonomous action triggers.
