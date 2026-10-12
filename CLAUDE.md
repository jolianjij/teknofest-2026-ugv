# CLAUDE.md — Tank-Drive Obstacle Course Robot

This file is project context for Claude Code. Read `docs/robot_project_plan.md` for full architecture/rationale before starting work — this file is the condensed operating reference.

## Project summary
A tank-drive robot navigates an obstacle-filled track: 2D lidar + Nav2 for obstacle avoidance, camera + onboard-NPU segmentation for drivable-terrain preference, and a timed crossing through a moving door. Two boards split the work: an ESP32 handles real-time motor control and sensor fusion, an **Orange Pi 5** handles navigation and perception.

**Board migration (2026-08-24):** the project moved from a Raspberry Pi 5 to an Orange Pi 5 (Rockchip RK3588, aarch64, Ubuntu 24.04 **Desktop**, ROS 2 Jazzy pre-installed). The Raspberry Pi is retired from this project. RPLidar C1 driver, `foxglove_bridge`, and the micro-ROS agent were all rebuilt/re-validated on the new board.

## Repo layout (create if it doesn't exist yet)
```
.
├── CLAUDE.md
├── docs/
│   └── robot_project_plan.md      # full plan — place the file the user already has here
├── teknofest_ws/                  # ROS 2 workspace, runs on the Orange Pi 5 (not git-tracked — Pi is source of truth)
│   └── src/
├── teknofest_ws_bridge/
│   └── serial_bridge.py           # ESP32 <-> ROS bridge node, deployed to ~/teknofest_ws/ on the Orange Pi
├── robot_description/             # URDF/xacro colcon package, authored here, deployed to ~/teknofest_ws/src/teknofest_description/ on the Orange Pi (same pattern as teknofest_ws_bridge)
│   ├── urdf/teknofest.urdf.xacro  # tank chassis, tracks, lidar, IMU, front/rear/aim cameras, aim pan/tilt chain - dimensions are placeholders, see file header
│   ├── launch/display.launch.py   # robot_state_publisher + joint_state_publisher_gui + rviz2
│   ├── package.xml
│   └── CMakeLists.txt
├── esp32_firmware/                # PlatformIO project (superseded, see esp32_firmware/PINOUT.md)
│   └── arduino_sketch/            # active firmware — Arduino IDE, plain serial protocol (see Architecture)
└── README.md
```

## Hardware
| Component | Role | Interface | Host |
|---|---|---|---|
| Orange Pi 5 | Nav/perception compute | SSH alias `orangepi` (**changed 2026-09-09**: now 192.168.137.7, not the previously-documented 192.168.1.106 — looks like a USB-tethered/RNDIS link, not the WiFi/LAN this project otherwise assumes; the connection was observed dropping and recovering within the same session, so treat it as flakier than a normal LAN link. DHCP, has changed before, check `~/.ssh/config` if unreachable; user `jolian`); VNC on display `:5` / port 5905 for desktop access | — |
| RPLidar C1 | Obstacle scanning, SLAM | USB | Orange Pi 5 |
| 3× USB cameras (front, rear, aim) | Front/rear drive view + target tracking (mounted on the pan/tilt aiming servo) — replaces the single Pi Camera v3 plan, satisfies the competition's 3-camera requirement | USB, via a powered hub (not bus-powered off the Orange Pi) — **⚠️ currently a USB 2.0 hub (QinHeng 1a86:8095, 480M), see Open Decisions "camera USB hub is USB2"** | Orange Pi 5 |
| Onboard RK3588 NPU | Terrain segmentation inference (~7 TOPS, driver already installed). No separate Coral accelerator — dropped in favor of this. | Built into the Orange Pi 5's SoC | Orange Pi 5 |
| ESP32 dev board (ESP32-WROOM-32 DevKitC) | Real-time control | USB — **now on the Orange Pi** (`/dev/esp32`, confirmed 2026-09-09; the "needs moving" note was stale) — flashing tool not yet set up there (no `arduino-cli`, only a bare ESP32 board index cached; ESP32 core + Adafruit libraries would need a fresh download, which this network has repeatedly throttled unpredictably) | — |
| BNO055 | Orientation fusion | I2C (`0x29` — this breakout's ADR pin is tied high, confirmed by I2C scan) | ESP32 |
| PCA9685 16-ch | PWM for ESCs/servos | I2C (`0x40`) | ESP32 |
| SSD1306-class OLED | Status display | I2C (`0x3C`) | ESP32 |
| State LED | 2-pin bi-color LED, system state indicator | GPIO 0 (red) / 2 (green) — ⚠️ both boot-strapping pins (see `esp32_firmware/PINOUT.md`). **Active again**: red solid = autonomous, green solid = manual, off = disarmed/failsafe | ESP32 |
| Pan/tilt aiming gimbal | Laser-designator aiming | **Differential mechanism** (confirmed 2026-08-26, "corrected" away 2026-09-07, reverted back 2026-09-09 — see Open Decisions for the full history; as of now `motorA = pan+tilt, motorB = pan`, so pure tilt moves motor A alone and pure pan moves both together), 2× NEMA17 + TMC2208 v1.2 steppers (STEP/DIR standalone mode, 32 microsteps via MS1/MS2), direct ESP32 GPIO — replaces the old PCA9685-servo pan/tilt entirely (ch 6/7 retired). Rate control, folded into the main firmware (`gimbal.h`/`gimbal.cpp`) — **not yet validated end-to-end on the physical gimbal**, see Open Decisions. | ESP32 (direct GPIO) |
| 2× drive motors + 8-channel relay board | Tank drive | **Changed (2026-08-31, revised same day)**: speed is a genuine analog voltage (1V–3.3V) via the ESP32's two DAC pins (GPIO25/26 direct, not PCA9685 — PCA9685 can only do PWM), plus relays for direction and braking, replacing the old dual-ESC (fwd/back channel) scheme entirely. See `esp32_firmware/PINOUT.md` PCA9685 channel table for the full relay map (also drives tank LEDs and the laser switch) — **relay channel mapping and polarity both inferred from description, not independently verified.** | ESP32→DAC (speed) + ESP32→PCA9685→relays (direction/brake) |
| FS-A8S (FlySky) RC receiver | Manual-run control input (Run 1) | SBUS, GPIO16 (UART2) | ESP32 |
| Emergency stop switch | Hardware panic-stop input, added 2026-09-07 | Normally-closed, one leg to GND, other to GPIO33 (`INPUT_PULLUP` — closed/normal reads LOW, open OR a cut wire reads HIGH and both trip it) — see Open Decisions "Independent e-stop hardware" for what this does and doesn't satisfy | ESP32 (direct GPIO) |

## Architecture
- **ESP32** = real-time spine: reads BNO055, converts `/cmd_vel` (autonomous) or SBUS (manual) to motor speed+relay outputs, drives OLED, LED, and the differential stepper pan/tilt gimbal, publishes IMU + diagnostics back to the Orange Pi. Talks over USB serial using a **plain text line protocol** (see `esp32_firmware/arduino_sketch/esp32_firmware/esp32_firmware.ino` header comment for the exact format) — **not micro-ROS**.
- **Orange Pi 5** = brain: `rplidar_ros`, 3× USB cameras (front/rear/aim; **decided**: `usb_cam` package, not `v4l2_camera`/`camera_ros`), `robot_localization` EKF fusing ESP32 IMU with lidar scan-matching, Nav2 planner/controller (MPPI preferred), and `serial_bridge.py` (`~/teknofest_ws/serial_bridge.py`, plain `rclpy` + `pyserial`, no colcon package) translating the ESP32's serial lines to/from ROS topics.
- **Vision strategy** (2026-08-24, proposed; sign detection + path segmentation scaffolded 2026-08-31, see below): three separate pipelines rather than one shared stack. **Front camera**: onboard-NPU (RKNN) segmentation for drivable terrain feeding the Nav2 semantic costmap (lidar stays authoritative for lethal obstacles), plus cone detection (lidar alone may miss cones depending on mount height/taper). **Door detector**: also NPU-accelerated, feeds a custom behavior-tree `IsDoorOpen` condition. **Aim camera**: classical CV (Hough circle/blob detection), not the NPU — the laser-fire target is a known high-contrast concentric-circle pattern, well suited to classical CV and avoids time-sharing the NPU between two models. **Rear camera**: no inference, just an operator feed + required footage recording.
  **`aim_tracker.py` control loop revised (2026-09-08).** Detection itself is unchanged
  (`target_detection.detect_target`'s nested-contour walk). What changed: the node used to feed the
  raw, unfiltered detection straight into a bare per-pixel proportional gain (`GAIN_PAN`/`GAIN_TILT`,
  0.006/px) - `target_detection.py`'s own `TargetFilter` and `PID` classes existed but were imported
  and never called (found during a bug-hunt pass, at the user's request). Now: `TargetFilter`
  (exponential smoothing + jump/miss rejection) sits between detection and control, and two `PID`
  instances (`ki=kd=0` for now - pure P, no hardware to tune the other terms against yet) operate on
  `pixels_to_degrees()`-converted error instead of raw pixels, so gains mean the same thing regardless
  of the aim camera's resolution. `PAN_KP=0.22`/`TILT_KP=0.17` were derived to be roughly equivalent
  to the old per-pixel gain at the aim camera's actual 2560x1440 profile and a **placeholder** FOV
  (`AIM_CAMERA_HFOV_DEG=70`/`VFOV=50`, not measured) - a continuity starting point, not a validated
  value. Also removed two dead-code items found in the same pass: an unused `clamp()` helper (PID's
  own `out_limit` already clamps), and three unused `RADIUS_RATIO_*`/`RATIO_TOLERANCE` constants that
  duplicated (and were shadowed by) `target_detection.py`'s real ones - editing them here used to have
  zero effect. **None of this is hardware-tested** - the closed-loop behavior, the FOV placeholder,
  and the KP values all need validation against the real aim camera and gimbal together.
  **Detection range extended toward 10m (2026-09-09).** Reported: target only detected out to ~5m,
  spec requires 10m. At the aim camera's configured 2560x1440, the target IS theoretically
  resolvable at 10m (~85px across, ~14px inner-ring radius - see `camera_manager_node.py`'s own
  PROFILES comment), so this was a detection-robustness gap, not a resolution ceiling: every
  measurement `target_detection.py` makes (circularity, radius ratios) gets intrinsically noisier
  on a smaller contour, and the thresholds were tuned/validated at closer range where that barely
  mattered. Loosened `CIRCULARITY_MIN` (0.45->0.35), `MIN_DETECT_CONFIDENCE` (0.70->0.58),
  `RATIO_TOLERANCE` (0.28->0.35), `MIN_RATIO_SCORE` (0.55->0.45); shrunk the blur kernel (5x5->3x3,
  less smearing of small rings) and CLAHE tile size (8x8->16x16, local contrast enhancement scaled
  closer to the target's own size instead of tiles larger than the whole target at range).
  **UNVALIDATED against real 10m footage** (no hardware access at the time) - re-run this module's
  original detect-rate validation methodology at 10m before trusting these; tighten back up if
  false-positive rate rises instead of detect rate improving.
- **Lidar front/back orientation fix (2026-09-09).** Reported via the dashboard: the lidar's own
  angle-0 reference does not line up with the vehicle's actual front - a mounting/frame issue in
  the raw `/scan` data, confirmed NOT a rendering bug (traced the dashboard's canvas math and it was
  already internally self-consistent - angle 0 correctly lands at the top of the canvas). This
  matters beyond the dashboard: `mission_manager.py`'s corridor-following reads the same raw angles
  for its forward/left/right sectors, so uncorrected this would have had the autonomous drive loop
  checking behind the vehicle instead of ahead of it. Added a single `LIDAR_FRONT_OFFSET_RAD =
  math.pi` constant in both `web_dashboard.py` (drawLidar) and `mission_manager.py`
  (`min_range_in_sector`, with proper angle-wrapping) - a clean 180 degrees is a best guess from the
  report, not a measured value. **If the real offset isn't exactly 180 degrees, change this one
  constant in both files rather than re-deriving the sin/cos signs.**
- **Camera recording (2026-09-09).** New "Record" button on the dashboard + new
  `teknofest_ws_bridge/mission_recorder.py` (launched as a subprocess by `web_dashboard.py`, one
  thread per camera matching `camera_manager_node.py`'s own capture-loop pattern) records
  front+rear+aim (720p) to `~/teknofest_ws/recordings/<timestamp>/`, alongside a `ros2 bag` of
  mission-relevant telemetry (IMU, diagnostics, `/scan`, `/cmd_vel`, mission/stage topics).
  Deliberately bypasses `camera_manager_node.py`'s exclusive one-camera-active model entirely -
  opens the three device paths directly. **Two real unknowns, unresolved without hardware access**:
  whether the USB bandwidth genuinely supports 3 simultaneous streams (camera_manager's own design
  assumes not, though a udev-rules comment elsewhere suggests the controller split needed for 2-at-
  once was done), and whether a second process can open a camera device a V4L2 driver already has
  streaming elsewhere. Both degrade gracefully - unavailable cameras are logged and retried, not
  fatal. Also wired `mission_manager.py` to auto-start/stop this same recording on the
  `/mission/active` rising/falling edge (a local HTTP call to the dashboard's own endpoint, fired
  from a background thread so a slow/unavailable dashboard can never block the drive loop) - "record
  the camera feed in the mission" no longer needs a manual click if the mission itself starts it.
  **Two real bugs found and fixed in a re-check the same day, before any of this ever ran on
  hardware**: (1) the capture device never had `CAP_PROP_FRAME_WIDTH/HEIGHT/FPS` set, only the
  FOURCC - the aim camera in particular would have opened at its native 2560x1440 and been
  downscaled in software after the fact, capturing far more USB bandwidth than the 720p profile was
  meant to use in the first place; (2) a camera reconnecting mid-recording opened a new
  `cv2.VideoWriter` on the SAME output path, which truncates it - meaning the exact failure this
  retry loop exists to survive (a camera hiccup) would have silently destroyed everything recorded
  before it. Fixed: the capture resolution/fps are now requested explicitly, and each reconnection
  writes to a new segment file (`front.mp4`, then `front_part2.mp4`, etc.) instead of overwriting.
- **Sign detection — SUPERSEDED (2026-09-03).** The production detector is now the user's own
  `ugv-stage-sign-detector` package (deployed to `~/teknofest_ws/src/ugv_stage_{interfaces,sign_detector}`,
  source of truth: `C:\Users\jolian\Downloads\UGV_Stage_Sign_Detector_v0.3.0\`). It is a hybrid:
  classical red-ring/circularity localisation feeding a small ONNX CNN, with 4-of-6 temporal
  confirmation, a course-order gate (`stage_manager`), and an `ExecuteStage` action boundary so
  perception never actuates the vehicle. **Validated on 4,300 frames of real printed-sign footage:
  all 12 signs confirmed in order at 0.88-0.96 confidence.** 46/46 tests pass natively on aarch64.
  **Stage-sequence reset added (2026-09-08, Claude, in the Windows source checkout only - NOT yet
  re-deployed to the Pi or re-tested on aarch64):** `stage_manager_node` originally had no way to
  reset `current_stage`, so a second autonomous attempt in the same process lifetime (disarm,
  reposition, re-arm) stayed stuck on whatever stage the previous attempt reached and rejected sign 1
  as out-of-order. Added `StageSequence.reset()` (`mission.py`) and a `/mission/reset_stage_sequence`
  (`std_msgs/Empty`) subscription on `stage_manager_node` that calls it - published once by
  `heading_reference.py` on every fresh manual→autonomous engagement. Deliberately does not publish a
  `StageTransition` (a reset isn't a sign-triggered behavior change). 48/48 tests pass locally
  (`test_mission.py` gained two reset tests) - needs recopying to `~/teknofest_ws/src/` and a colcon
  rebuild on the Pi before it's live there.
  **`unknown → stop` (2026-09-03):** label 0 used to be silently discarded, so a sign the model
  could not read was indistinguishable from no sign and the vehicle drove past a known instruction.
  It is now published as sign 0 and `stage_manager` halts on it *without advancing the sequence*.
  Guards against nuisance stops: the candidate must already pass red-ring geometry, and must clear
  `unknown_min_geometry` (classifier confidence is meaningless here — failing it is what "unknown"
  means). Measured rate on the real footage: **1 frame in 4,300**. Config: `unknown_stops_vehicle`,
  `unknown_min_geometry` in `config/detector_ml.yaml`.
  **⚠️ The template fallback (`detector.yaml`) has no reject class** — it always returns its
  best-matching digit, so it can never report unknown and this fail-safe does not exist in that
  mode. Do not run template mode as the competition configuration.
  The old synthetic-CNN/YOLO experiments in `vision_models/sign_classifier/` are superseded and kept
  only for reference; they never worked on real camera input.
- **Sign detection — superseded, historical** (`vision_models/sign_classifier/`, 2026-08-31): a trained CNN, not classical OCR — per explicit decision, sign identification had to be a trained model. Classical red-ring/circularity color detection (`teknofest_ws_bridge/sign_detector.py`) still does per-frame *localization* (cheap, runs every frame); a small CNN (`SignCNN`, ~64×64 input, CPU inference via onnxruntime, NOT the NPU — NPU stays reserved for terrain segmentation) then classifies each candidate ROI into 11 classes (0=not a sign, 1-10=stage number). Trained entirely on procedurally-generated synthetic renders of the known sign design (`dataset.py`) — no real sign photos exist yet, but the design (red ring/white fill/black digit, exact geometry) is fully specified, so synthetic data is a legitimate substitute here specifically (unlike terrain, see below). Trained with the IOAI2026 project's venv (`C:/Users/jolian/Desktop/IOAI2026/.venv`, has torch+cuda/onnx/onnxruntime already). **Not yet validated against a real printed sign or real camera frame** — synthetic-only accuracy is not a stand-in for that, per the hardware-validation policy below. Synthetic validation (2026-08-31): CNN is 98.9% accurate on a held-out synthetic set, and 100% (18/18) in an end-to-end localizer+CNN test whenever the classical localizer actually proposed a candidate ROI — **but that localizer missed the sign entirely in ~40% (12/30) of fresh synthetic test scenes**, almost certainly its pre-existing untuned HSV red-ring thresholds reacting to synthetic lighting/color jitter. This localizer-recall gap (not CNN accuracy) is now the likely real-world bottleneck — expect it to need the same real-lighting threshold tuning already flagged for it, before trusting detection range/recall on the physical sign.
- **Front camera now carries three duties (2026-09-03)**, all deployed: (1) stage-sign reading,
  (2) drivable-terrain + cone obstacle extraction (`cone_terrain.py` → `/front/ground_obstacles`,
  range/bearing under a flat-ground projection — **camera height 0.45 m and pitch −8° are
  PLACEHOLDERS, measure before trusting any range**), (3) sliding-door direction
  (`door_tracking.py` / `door_direction_node.py` → `/door/state`). Door tracking verified on
  synthesised sweeps only: 32/33 and 31/31 correct on right/left, zero misreads, nothing reported on
  a static scene, period 1.83 s vs 1.83 s truth. **Never seen a real door.** Lidar stays
  authoritative for whether the gate is passable; the camera's unique contribution is direction.
- **Path/road segmentation — TRAINED, but on the wrong scene (2026-09-03).** The full pipeline is
  proven end to end: `capture_frames.py` (on the Pi) → `make_pseudo_labels.py` (heuristic
  auto-labels, no manual annotation) → `train.py` (Windows RTX 4060 via the IOAI2026 venv) → ONNX →
  deployed. `path_segmentation_node.py` loads the CNN and publishes `/front/drivable_mask` at 30 Hz
  and `/front/ground_obstacles` at 29 Hz.
  **The current checkpoint is NOT usable for the competition.** It was trained on 322 frames from a
  stationary camera pointed at an indoor room — no ground plane in view, no viewpoint variety. Its
  val IoU of 0.982 only means the CNN faithfully imitated the heuristic on one scene (its output
  drivable fraction, 0.513, matches the heuristic's 51.3% exactly). Retraining on real course
  footage is now a three-command job. Also note `train.py` originally looked for masks using the
  image's own `.jpg` extension while masks are written as `.png` — fixed.
- **Path/road segmentation — original scaffold** (`vision_models/path_segmentation/`, 2026-08-31): unlike the sign, off-road terrain appearance can't be procedurally synthesized convincingly, so there's no synthetic-data shortcut — training a real CNN needs real front-camera frames of actual course-like terrain, which don't exist yet (camera bring-up still blocked on the USB2 hub, see Open Decisions). Two-tier design: (1) `ground_plane_heuristic.py` — a classical, **zero-training** SLIC-superpixel + seed-region color-similarity heuristic, deployable today as soon as `/front/image_raw` exists, and also the intended pseudo-label source for (2); (2) `cnn_model.py`/`train.py` — a MobileNetV3-Small-encoder + LR-ASPP-style decoder binary segmentation net, ONNX-exportable for RKNN conversion, **scaffolded but not trained** (no dataset directory exists yet). `teknofest_ws_bridge/path_segmentation_node.py` auto-selects the CNN if a checkpoint is present, else falls back to the classical heuristic. As always, lidar stays authoritative for lethal-obstacle avoidance regardless of what either backend outputs.

## Tech stack & conventions
- OS: Ubuntu 24.04 Desktop (arm64) on the Orange Pi 5 — has a full GNOME/XFCE desktop + VNC (TigerVNC) for direct access, unlike the original Pi 5 plan.
- ROS 2 Jazzy. Build `teknofest_ws` with `colcon`.
- Language: Python for ROS 2 nodes unless a specific loop needs C++ performance.
- ESP32: Arduino-ESP32 core, plain serial text protocol (no micro-ROS — see Open Decisions). Active firmware lives in `esp32_firmware/arduino_sketch/esp32_firmware/`, flashed via Arduino IDE. The `esp32_firmware/` PlatformIO project (root of that folder) and `micro_ros_arduino`/`micro_ros_platformio` are both superseded/unused now.
- Remote monitoring: `foxglove_bridge` for ROS topic/sensor visualization. VNC (TigerVNC, systemd-enabled, display `:5`) is also set up for general desktop access to the Orange Pi — not a replacement for foxglove_bridge, the two serve different purposes.
- DDS: CycloneDDS, tuned for Wi-Fi (see plan doc §"DDS over WiFi" — pin the interface, limit multicast, raise receive buffers).
- Commit at the end of each roadmap stage, not mid-stage.
- **This network throttles/blocks large downloads unpredictably** (seen against GitHub raw content, GitHub git-over-HTTPS via HTTP/2, Microsoft's VS Code CDN, default Ubuntu mirror geo-routing, and PlatformIO's toolchain host). Known working mitigations: `raw.githubusercontent.com` → try `cdn.jsdelivr.net/gh/<user>/<repo>@<ref>/<path>` instead; git HTTP/2 stream errors (`curl 92`) → `git config --global http.version HTTP/1.1`; stalled/hung downloads → check `/proc/<pid>/io` `read_bytes` across two samples to confirm a real stall (not just slow) before killing/retrying.

## Deployment on the Orange Pi (2026-09-03 — first full bring-up)
Everything below runs from `~/teknofest_ws/` on the Pi. Each has a detached start script; launch with
`ssh -f orangepi "setsid ~/teknofest_ws/<script> > /tmp/<name>.log 2>&1 < /dev/null"` — a plain
`nohup ... &` over SSH dies with the session.

| Script | Starts | Verified |
|---|---|---|
| `start_cameras.sh` | `apply_camera_settings.sh` then `camera_manager_node.py` | front ~19-29 Hz |
| `start_lidar.sh` | `rplidar_ros` on `/dev/rplidar` | `/scan` 10 Hz, health OK |
| `start_serial_bridge.sh` | `serial_bridge.py` on `/dev/esp32` | `/esp32/imu` 20 Hz |
| `start_sign_stack.sh` | sign detector (ONNX) + stage manager | running, `unknown→stop` armed |
| `start_segmentation.sh` | `path_segmentation_node.py` | mask 30 Hz, obstacles 29 Hz |
| `start_dashboard.sh` | `web_dashboard.py` | http://192.168.1.106:8080/ |
| `start_foxglove.sh` | `foxglove_bridge` | ws://192.168.1.106:8765 |

**⚠️ `/dev/rplidar` and the ESP32 symlink were CROSSED and this cost hours.** Both devices are
Silicon Labs `10c4:ea60`, and the old `99-rplidar-c1.rules` matched that VID/PID with **no serial
qualifier**, so it grabbed whichever enumerated first. The serial bridge was talking to the lidar
(silent unless commanded) and the lidar driver to the ESP32 (doesn't speak RPLidar protocol) — both
looked like dead hardware. Also note the chip mapping is the opposite of the intuitive guess:
**CP2102 (serial `0001`) = ESP32**, **CP2102N (serial `b05ab…`) = RPLidar**. Identify by protocol,
never by chip name. Both are now serial-qualified in `/etc/udev/rules.d/99-teknofest-serial.rules`.

**Camera udev is now keyed on USB port**, not model name — both drive cameras are the same
Fantech C30, so `ATTR{name}` matching made `cam_front` a coin flip. See
`/etc/udev/rules.d/99-teknofest-cameras.rules`: `7-1.2`=rear, `7-1.3`=aim (C50), `7-1.4`=front.

**Camera controls do not persist** across replug and `usb_cam`'s own parameter names don't match this
kernel's controls (it logs "unknown control" and continues). `apply_camera_settings.sh` sets them via
`v4l2-ctl` and must run before any camera node. Front/rear were found at `brightness=50` against a
default of 128 — that alone is why every earlier frame looked almost black; now 140 with `gain=40`.
C50 autofocus is off with `focus_absolute=289` (uncalibrated, but stable — AF hunting mid-engagement
would blur the laser target's rings).

**The Pi's OpenCV 4.11 has NO `cv2.ximgproc`.** Anything relying on contrib (SLIC/SEEDS/LSC
superpixels) will `ImportError` on the target hardware. `ground_plane_heuristic.py` was rewritten to
base-OpenCV only for this reason.

## Current stage
**Stage 0 done. Stage 2 core link verified end-to-end (2026-08-24). Stage 1 camera bring-up in progress (2026-08-25).** RPLidar C1 built, `/scan` publishing, `foxglove_bridge` running on the Orange Pi. ESP32 firmware (BNO055, PCA9685, SBUS) is flashed and confirmed working over USB serial to `serial_bridge.py` on the Orange Pi: `/esp32/imu` publishes real data at ~18.6 Hz, `/esp32/diagnostics` publishes, `/cmd_vel` writes through without error (not yet tested with SBUS armed, so no confirmed physical motor response). Stages are defined in `docs/robot_project_plan.md` §7:
0. Pi bring-up (Ubuntu + ROS2 Jazzy + foxglove_bridge + RPLidar C1 `/scan`) — **done**
1. Cameras (front/rear/aim) + onboard NPU (RKNN) drivers validated independently — **partially done (2026-08-25):** `usb_cam` installed, stable udev symlinks (`/dev/cam_front`, `/dev/cam_aim`) created via `ATTR{name}` matching. C50 autofocus disabled (`focus_absolute` locked at 289, uncalibrated). Front (C30) camera publishes reliably standalone at 640×480 YUYV, ~30 Hz. Aim (C50) camera also publishes reliably standalone at 640×480 YUYV (~13-23 Hz, lower and burstier than front for unknown reasons — not yet investigated). **Blocker: front + aim cannot run simultaneously** — see Open Decisions "camera USB hub is USB2". `pixel_format: "mjpeg2rgb"` reproducibly segfaults on this `usb_cam` 0.8.1 build regardless of resolution (real driver bug, not our config) — use `"yuyv"` instead, not `"mjpeg2rgb"`. `pixel_format: "raw_mjpeg"` (compressed passthrough, would dodge both the segfault and the bandwidth ceiling) also crashes on this build (`terminate called after throwing an instance of 'char*'`) — not usable either. Rear camera (3rd unit) still not sourced/connected. NPU segmentation model work not started.
2. ESP32 serial link + BNO055 publishing — **verified working** (see above). Note: `/esp32/odom` from the original comm-topics plan is not implemented by the current serial protocol/bridge (only `/esp32/imu` + `/esp32/diagnostics`) — revisit if the EKF needs it.
3. Motor path (PCA9685 + ESC calibration, wheels off ground) — firmware written, **ESC pulse widths are still placeholders**, not calibrated, not tested, SBUS receiver not yet connected/armed for a physical test
4. OLED status — **verified working on hardware.** LED status — **verified working** (red=autonomous, green=manual)
5. Core navigation (Nav2 + EKF + lidar-only avoidance) — **sim scaffolding exists
   (`robot_description/`), never run/confirmed, and was undocumented here until now.**
   `config/ekf.yaml` + `config/nav2_params.yaml` + `launch/nav2_sim.launch.py` build a full
   Gazebo-sim Nav2 stack: no `map_server`/AMCL (deliberate — the competition course is
   unknown until race day, so both costmaps use `odom` as their global frame and Nav2 runs
   purely reactively off the rolling local costmap + lidar), MPPI controller, NavFn planner,
   behavior server, collision monitor. **2026-09-11: added the real-hardware counterpart** —
   `config/ekf_hardware.yaml` + `config/nav2_params_hardware.yaml` + `launch/nav2_hardware.launch.py`,
   identical design with `use_sim_time: false` and `rf2o_laser_odometry` (2D frame-to-frame
   scan matching, no map needed) standing in for the wheel encoders this vehicle doesn't have.
   **Entirely unvalidated in every direction**: the sim stack has never been launched even
   once; `rf2o_laser_odometry` has never seen this lidar's real scan data; and — the one that
   actually matters before touching either — **this has never been run alongside
   `mission_manager.py`'s own Phase 1 reactive corridor-following, and the two are NOT
   currently reconciled**: both publish `/cmd_vel` directly, neither defers to the other.
   Launching `nav2_hardware.launch.py` is a replacement for `mission_manager.py`'s driving
   role, not a complement to it — running both at once means two systems fighting over the
   same topic. Decide which owns driving (or how they hand off) before wiring either into
   `start_all.sh`.
6. Vision costmap (segmentation layer) — **scaffolded, not trained/tested (2026-08-31)**: see "Path/road segmentation" under Architecture. Classical fallback (`ground_plane_heuristic.py`) is code-complete and needs no training, but untested against real footage (front camera not yet reachable together with the Orange Pi). CNN path needs real course-terrain frames before it can train at all.
7. Moving door (detection + BT subtree + dynamic costmap)

## Working agreement
- Work stage by stage, in order. Don't jump ahead to later stages while earlier ones are unvalidated.
- **This is physical hardware.** Never mark a hardware-dependent step as done from code alone (e.g. "motor spins correctly," "heading matches physical rotation," "ESC calibrated") — ask the user to confirm it in person after each such step.
- Don't silently resolve items in Open Decisions below — propose a specific default and get confirmation before wiring or flashing anything based on it.
- Flag pin/I2C-address conflicts immediately, don't guess around them.
- Keep this file and `docs/robot_project_plan.md` in sync as decisions get finalized (e.g. once serial pins are chosen, update both).

## Open decisions — ask, don't assume
- ~~Serial link~~ — **Decided:** USB serial (ESP32 DevKitC onboard USB-UART bridge), 115200 baud, plain text line protocol.
- ~~State LED~~ — **Decided:** 2-pin bi-color LED, GPIO 0 (red) / 2 (green) — actual physical wiring, confirmed by the user. Both are boot-strapping pins (GPIO0 especially — must read HIGH at reset or the chip enters UART download mode); this was the root cause of a long "board won't boot/transmit anything" debugging session on 2026-08-24. Safe during normal operation, only matters at reset. See `esp32_firmware/PINOUT.md`. User chose to keep this wiring rather than move to GPIO25/26 — revisit if boot flakiness recurs. Re-enabled 2026-08-24 with a simplified pattern: red=autonomous, green=manual.
- ~~FlySky channel count~~ — **Decided:** i6 transmitter flashed with the community 10-channel mod (CH7-10 unlocked). CH5/6 = VarA/VarB (aiming pan/tilt), CH7/8 = SwA/SwB (arm/mode). CH9/10 (SwC/SwD) still unassigned.
- Encoders: none currently planned — revisit if Stage 5 odometry is too poor
- ESC calibration values (neutral/forward/reverse pulse widths) — **placeholders in `esp32_firmware/include/config.h`, must be measured on the actual ESCs before enabling motors**
- Power budget/rail separation — pending ESC current draw
- ~~micro-ROS vs custom serial protocol~~ — **Decided 2026-08-24: custom serial protocol, not micro-ROS.** Tried both `micro_ros_platformio` (build script Windows-incompatible) and `micro_ros_arduino` (precompiled, matched ROS distro, correct wiring/I2C addresses) — the agent never completed a session with the ESP32 despite the serial link demonstrably carrying well-formed, correctly-parsed frames (confirmed via `strace` on the agent). Root cause never identified after extensive debugging. Switched to a plain text line protocol + `serial_bridge.py` (`rclpy`+`pyserial`) instead — verified working end-to-end same day. Revisit micro-ROS only if a real need for it emerges (e.g. QoS/DDS features the custom bridge can't provide).
- **Drivetrain sizing** — competition spec requires climbing 45% grades and fording 40cm water at 1.2–2m vehicle scale; current "2× bidirectional BLDC + ESC" has no torque/power spec yet, needs validation against real motor datasheets
- **Independent e-stop hardware** — competition spec requires e-stop/comms-loss detection isolated from the main control unit. Firmware implements a software+PCA9685-`!OE` kill switch (see `esp32_firmware/PINOUT.md`), which is a partial mitigation, not full compliance — a hardware safety loop outside the ESP32 is still undesigned.
  **Physical NC e-stop switch added (2026-09-07)**: one leg to GND, other to GPIO33, read via `INPUT_PULLUP` in `esp32_firmware.ino` — closed (normal) reads LOW, open (pressed) OR a cut/disconnected wire both read HIGH and both trip it, which is the fail-safe direction you want. Folded into the existing `armed` boolean in `loop()`, so it rides the same path as the SBUS arm switch: the PCA9685 `!OE` hardware kill, brake-on-disarm, and the gimbal's `armed`-gated enable all already cut out correctly with no separate wiring needed. New `RobotState::ESTOP` (both status LEDs blink together, OLED reads "E-STOP"). **This is still read by the ESP32's own firmware, not an independent hardware loop that cuts power directly** — it stops the vehicle reliably through software, but does not by itself satisfy the competition's "isolated from the main control unit" requirement any more than the PCA9685 `!OE` kill did. **Not yet hardware-tested** — wire it and confirm in person (per the working agreement) that pressing it actually stops the vehicle and that the LED/OLED reflect ESTOP correctly.
- **Aiming gimbal stepper integration** (2026-08-31, code-complete, NOT hardware-validated) — ~~differential stepper mixing (`tilt = motorA − motorB`, `pan = motorB`, confirmed empirically via the standalone `differential_gimbal_test/` sketch)~~ **WRONG, see the 2026-09-07 "Mixing model correction" entry further down — this is not a differential mechanism, motor A drives tilt only and motor B drives pan only, no cross-term.** This mixing (whichever formula was current at the time) is folded into the main firmware as `gimbal.h`/`gimbal.cpp`, replacing the PCA9685 ch6/7 servo pan/tilt code entirely. Control is now **rate control** (stick deflection → angular velocity, with acceleration ramping via `GIMBAL_ACCEL_STEPS_S2`), not absolute position — open-loop steppers have no position feedback. Soft position limits (`GIMBAL_PAN_LIMIT_STEPS`/`GIMBAL_TILT_LIMIT_STEPS`) exist but are in raw steps, not degrees, since the gear/differential ratio isn't measured yet — user is aware and will measure it later. `GIMBAL_MAX_STEP_RATE` (2000 steps/sec) is still an untuned guess. **Two real pin conflicts found and fixed in this second wiring round**: shared EN is GPIO14 (not the user's stated GPIO4, which collides with the PCA9685 `!OE` drive-motor kill switch — confirmed by the user's own symptom report: "works grounded, doesn't work on GPIO4"); tilt STEP is GPIO27 (not GPIO17, which the main firmware's SBUS UART2 peripheral claims via `sbus.begin()` regardless of physical wiring — user initially pushed back on this as unnecessary since SBUS_TX is electrically unused, but it's a peripheral-ownership conflict, not a physical one; GPIO23 was tried first but isn't accessible on this board/harness, landed on GPIO27). Final pins: EN=14, pan STEP=18/DIR=19, tilt STEP=27/DIR=5. Requires MS1=HIGH/MS2=LOW wired on both TMC2208 drivers for the assumed 32 microsteps — confirm this is actually wired.

  **Control scheme revised a second time (2026-08-31)**: pan/tilt now reuses the steer/throttle sticks as direction inputs (steer→pan, throttle→tilt) instead of a dedicated joystick, gated by SwC (CH9) as a **3-position** switch: off = drive normally; middle = aim mode (drive forced to zero, sticks become pan/tilt); full = aim mode + laser relay (`PCA_CH_LASER`) on. Read via `SBUS_3POS_LOW_THRESHOLD`/`SBUS_3POS_HIGH_THRESHOLD` (config.h) splitting the raw SBUS range into thirds. VarA/VarB came back, but as **speed dials** (0..1, scaling `GIMBAL_MAX_STEP_RATE` per axis), not direction inputs. SwD (CH10) is now brake — forces drive to zero regardless of SwC, but does NOT disable aim mode (braking in place shouldn't block aiming). This whole scheme, including the SwC 3-position semantics, is Claude's interpretation of the request, not independently confirmed by the user — verify it matches intent before relying on it.

  Laser module itself still not designed. **Nobody has confirmed the physical gimbal responds correctly to stick input yet — this is a code change, not a hardware confirmation, per the working agreement.**

  **Follow-up fixes (2026-08-31)**: tilt direction was inverted on real hardware (stick down moved tilt up) — negated in `esp32_firmware.ino`. `GIMBAL_ACCEL_STEPS_S2` lowered from 4000 to 600 and a temporary `G,` serial debug line added (prints `panSpeedScale`/`tiltSpeedScale`/`panRate`/`tiltRate`) to verify the speed-dial and acceleration ramp are actually doing something, since at the original value the ramp completed in a fraction of a second and could've read as "not working" either way — not yet confirmed which it was. Remove the debug line once confirmed.

  **One-axis-at-a-time rule added (2026-09-08)**, per explicit user request for laser-aim tuning: if
  both `panStick` and `tiltStick` are nonzero in the same `gimbalUpdate()` call, only the
  larger-magnitude one is honored - the other is forced to zero for that update (`gimbal.cpp`). This
  is an operating rule, not a mechanism limit - the two axes genuinely can move together (see the
  correction below) - it exists purely because diagonal motion makes it hard to isolate which axis's
  gain is producing what you're seeing while tuning, and makes fine aim corrections during an actual
  shot less predictable. Applies uniformly to manual stick reuse AND the autonomous
  `A,<pan>,<tilt>,<fire>` command (`aim_tracker.py`'s output), since both funnel through the same
  function - `aim_tracker.py` needed no change. Uses hysteresis (`DOMINANCE_HYSTERESIS = 1.2`, the
  challenger axis must exceed the incumbent by 20%) rather than a plain magnitude comparison, since a
  bare "bigger wins" check would flip the winning axis every loop whenever the two are near a tie (a
  diagonal stick position, or noisy vision-tracking error) - **not yet hardware-confirmed** that the
  hysteresis margin is right; watch for either flicker (lower it) or sluggish axis-switching (raise
  it) when tuning on the real gimbal.

  **Mixing model correction (2026-09-07): it's not a differential.** The 2026-08-26 confirmation above (`tilt = motorA − motorB, pan = motorB`) was wrong for this hardware. The user confirmed on the actual gimbal that motor A drives tilt only and motor B drives pan only, with no cross-coupling — two independent axes, not a differential mechanism. The old formula commanded `motorA = pan + tilt`, which bled the pan command into the tilt motor and was the actual cause of a reported "can't move pan and tilt at the same time" symptom (moving both together made the tilt motor's actual behavior reflect pan too, not pure tilt). Fixed in `gimbal.cpp`/`gimbal.h`: `motorA = tiltCurrentRate`, `motorB = panCurrentRate`, no summing. **Not yet hardware-confirmed** that this fully resolves independent simultaneous motion — needs an in-person retest per the working agreement.

  **Reverted back to differential (2026-09-09).** The 2026-09-07 correction directly above is itself now superseded. The user re-described the hardware's actual behavior in different words - "tilt moves only on tilt while the other one is locked, for pan you need to move both" - which is unambiguously the ORIGINAL 2026-08-26 differential formula (pure tilt: only motor A moves, motor B locked at 0; pure pan: both motors move together in lockstep), not the 2026-09-07 "two independent axes" model. Reverted in `gimbal.cpp`/`gimbal.h` to `motorA = panCurrentRate + tiltCurrentRate`, `motorB = panCurrentRate`. This is the THIRD time this exact mechanism has been described, twice in mutually-contradictory terms across two sessions - the mixing model itself is not in doubt at the code level (it directly encodes whatever was most recently described), but given the flip-flopping, **do not trust either description over an actual in-person test the next time the gimbal is powered** - watch what genuinely happens when commanding pure pan vs. pure tilt before doing anything else with it (tuning, autonomous aim+fire, etc.).

  **Tilt-only drift compensation added (2026-09-09).** Reported: pan never drifts, tilt drifts a
  bit. Pan not drifting while tilt does is consistent with a MECHANICAL cause specific to the tilt
  linkage (backlash, or motor A alone fighting gravity/load while pan drives both motors in
  lockstep), not a firmware bug - and an open-loop stepper with no position feedback cannot detect
  that it has drifted, so nothing in firmware can correct for it directly. Added a bounded,
  opt-in workaround instead: `TILT_IDLE_DRIFT_COMPENSATION_STEPS_S` (`config.h`, defaults to `0.0`,
  no effect) adds a small constant step-rate bias to motor A ONLY while tilt is idle
  (`gimbal.cpp`). **Needs the real drift direction/rate observed on hardware before it's useful** -
  tune by raising the magnitude from 0 in small steps (~20 steps/sec) with the sign matching
  whichever direction counters the drift; a wrong sign makes it visibly worse immediately, so this
  is safe to experiment with live. If the real cause turns out to be a Vref/current setting on the
  TMC2208 or physical play in the mechanism, this constant should stay at 0 and the real fix
  happens in hardware instead.

  **Root-caused a stepper-speed bug (2026-09-07): the main `loop()` was starving `gimbalUpdate()` of the call rate it needs.** `gimbal.cpp`'s `serviceAxis()` emits at most one step pulse per raw motor per call — by design, so it never blocks the safety checks — which means the *achievable* step rate is capped by how often `loop()` actually runs, not just by the commanded rate. `loop()` was doing ~8 unconditional PCA9685 I2C writes every single iteration (both direction relays, both brake relays, both brake servos, the laser, the tank LEDs) regardless of whether any value had changed, plus an unconditional BNO055 Euler read every iteration for a display that only redraws at 5Hz — enough I2C bus time per loop to cap the real step rate well below `GIMBAL_MAX_STEP_RATE` (2000 steps/sec needs a 500µs loop; the I2C traffic alone likely cost several ms). Fixed by: (1) `Wire.setClock(400000)` — 400kHz instead of the 100kHz default, cutting every transaction to about a quarter of its time; (2) `pcaDigitalWrite` (`relays.h`) and `setBrakeServoAngle` (`esp32_firmware.ino`) now cache the last value written per channel and skip the `setPWM` call entirely when nothing changed, turning ~8 writes/loop into ~0 in steady state; (3) the Euler read moved inside the same 200ms throttle as the OLED redraw instead of running every iteration. **Not yet hardware-confirmed** — the diagnosis is grounded in the code (the loop-vs-step-rate math is unambiguous) but per the working agreement this needs an in-person check that the gimbal now tracks the speed dial properly, especially at higher `panSpeedScale`/`tiltSpeedScale` settings where the old loop rate would have clipped it hardest.

  **Autonomous aim+fire hook added (2026-08-31, ESP32-side only)**: when SwB is autonomous AND SwC is in an aim position (middle or full — both equivalent here), pan/tilt direction AND the laser relay both come from a new serial command `A,<pan>,<tilt>,<fire>` (Pi→ESP32, mirrors `C,<linear>,<angular>`'s staleness-timeout convention, fails safe to off if stale) instead of the sticks/switch. Intended workflow, per the user: drive the whole track manually, then at the shooting area flip SwC to middle and SwB to autonomous — the vision system takes full aim+fire authority at that point since nobody's holding a stick to aim. This is purely the firmware hook — no Pi-side sender exists since the actual vision-based autonomous targeting (camera → target position → aim+fire command) is deferred vision/ROS work, not built yet. Outside that specific state, the laser still fires only via SwC's full position under direct operator control, unchanged. Autonomous driving is also blocked while SwC is in an aim position (mirrors manual).

  Note: an earlier version of this entry had the laser always gated by SwC's full position regardless of manual/autonomous, keeping fire authority with the human even in autonomous mode — the user corrected this, clarifying autonomous+aim-position should hand full fire authority to the vision system.

- **Drivetrain redesign: relay board replaces dual-ESC scheme** (2026-08-31, code-complete, NOT hardware-validated) — motors changed from 2 unidirectional ESCs each (fwd channel + back channel) to one speed signal per motor plus an 8-channel relay board for direction (reverse-enable relays) and braking (dedicated brake relays), plus 2 more relay channels for tank LEDs and the laser switch, 2 spare. Relay board IN1–IN8 assumed wired to PCA channels 15 down to 8 (descending) — **this ordering is inferred from the user's description, not independently verified**, confirm before trusting any specific relay's behavior. `RELAY_ACTIVE_LOW` (config.h) assumes the common cheap-relay-board convention (LOW = energized) — also unverified against this specific board. The brake switch is SwD/CH10 (not SwC, which is the 3-position aim switch — see the gimbal entry above); brakes also auto-engage on disarm/failsafe as a safety default Claude added, not explicitly requested — reconsider if unwanted. **This also changed the ESP32↔Pi serial protocol's `S,` line semantics** (see Comm Topics table) — `serial_bridge.py` on the Pi needs a matching update before its `/esp32/motor_pwm` topic is trustworthy again.

  **Brake/LED remapping (2026-09-09, code-complete, NOT hardware-validated).** The single shared
  SwD brake switch is gone. Per-side manual brake now lives on two previously-unused stick channels
  (`SBUS_CH_RIGHT_BRAKE`=CH1, `SBUS_CH_LEFT_BRAKE_SERVO`=CH4, `config.h`), each with a large
  (`BRAKE_CHANNEL_DEADZONE`=0.20, vs. 0.10 for the drive sticks) deadzone specifically so stick drift
  at rest can never trigger a brake. Deliberately asymmetric, per explicit request: CH1 thresholds the
  RIGHT relay+servo together (on/off, same as the old shared-switch behavior, just per-side); CH4
  gives the LEFT SERVO genuine proportional control (0..1 maps linearly across its engaged/released
  angle range) while the left relay is thresholded off that same channel value. SwD itself now drives
  the tank LEDs relay (`PCA_CH_TANK_LEDS`) directly as a manual switch, replacing the earlier
  "lit while driving" automatic heuristic entirely - not layered under it. **Mechanical brake servos
  no longer engage during the direction-reversal hold** (explicit request: "remove the mechanical
  brake when reverse") - the electrical relay above still does, since that's what actually makes
  reversal work (see the driver constraint below); the servo's role is now only the manual channel and
  disarm/failsafe. Brake servo PCA9685 channels also moved again: right=0, left=5 (was 6/7) - synced
  into `brake_servo_calibration.ino` too, which had drifted out of sync with `pins.h` (comment said
  channel 5, constant said 6 - fixed both to agree). Separately confirmed by the user: **`PIN_PCA9685_OE`
  is not physically wired on this build** - the firmware still drives it, harmlessly, but this specific
  hardware kill-switch layer provides zero actual protection right now; brake-on-disarm and the
  physical e-stop switch are what actually work. **None of this has been tested on the real vehicle.**
  **Speed signal corrected a second time (2026-08-31)**: the motor driver turned out to take a genuine analog voltage (1V–3.3V active range, confirmed by the user's own testing — below 1V and above 3.3V both did nothing), not PWM or an RC-servo pulse. The original implementation used `pca.writeMicroseconds()` (an RC-servo-style ~5-10% duty pulse at 50Hz) which would have been wrong for this driver regardless of the PCA9685 vs. direct-GPIO question. Fixed by moving speed output onto the ESP32's two genuine DAC pins — **GPIO25 and GPIO26 are the only pins on this chip capable of true analog voltage output** via `dacWrite()`; no PCA9685 channel can do this since it only ever produces PWM. Right motor is GPIO26; left motor is GPIO25, not the user's originally requested GPIO27 — GPIO27 has no DAC hardware at all (independent of it also already being the gimbal's tilt STEP pin). `DAC_MIN_VALUE`(79)/`DAC_MAX_VALUE`(255)/`DAC_OFF_VALUE`(79) in `config.h` map 0..1 motor magnitude onto the 1V-3.3V range. **Correction (2026-09-11, confirmed on real hardware, do not "fix" this again without a fresh test):** this originally kept `DAC_OFF_VALUE` at literal 0V, distinct from `DAC_MIN_VALUE`'s ~1V floor - reasonable-looking from the code alone, and briefly "fixed" back to that distinction the same day when a stale value (79, same as `DAC_MIN_VALUE`) was found. Flashed and tested: at literal 0V the motor driver stopped responding to any command at all ("now it dont move"). This specific driver needs the speed line held at ~1V even at rest to stay responsive - reverted `DAC_OFF_VALUE` back to matching `DAC_MIN_VALUE`, which is what it should stay at. The old `ESC_PULSE_OFF_US`/`ESC_PULSE_MAX_US` constants and the now-dead `normalizedToDualEscPulses()`/`DualEscPulses` (motor_mixer.h/.cpp, from the original dual-ESC scheme) were removed entirely rather than left as unused dead code.
- **Individual brakes, reverse behavior, and turn oscillation (2026-09-11, code-complete, NOT hardware-validated — the ESP32 has never been flashed with any of this).** First real-hardware run of `mission_manager.py`'s corridor-following: **it drove straight between two walls correctly**, but oscillated between forward/reverse (real torque loss on this driver each time) and didn't handle turns. Four fixes, all from direct hardware reports:
  1. **Per-side brake bug** — `esp32_firmware.ino` was ORing the two manual brake channels into one combined flag that zeroed BOTH wheels' drive output the instant either was active, even though the brake relay/servo logic further down was already correctly per-side. Fixed: each side's own brake channel now only zeroes that side's own drive output.
  2. **Reverse always full magnitude** — explicit request: "always reverse on max speed in all cases in auto and manual because reverse is slow and fixed speed." `applyMotorOutputs()` now uses magnitude 1.0 whenever a wheel is reversing, in both MANUAL and AUTONOMOUS, instead of a proportional fraction.
  3. **Reversal ramp** — after the existing brake+zero-voltage hold on a direction flip, speed now eases from `DAC_MIN_VALUE` up to the commanded value over `REVERSAL_RAMP_MS` (500ms, `config.h`) instead of snapping to full torque immediately.
  4. **No reverse from steering (the actual oscillation root cause)** — `motor_mixer.cpp`'s `mixDifferential()` used to let a sharp/continuous turn (`angular > linear`) compute a negative mix on the inside wheel. Combined with fix #2 (reverse always at max) and the brake+hold+ramp cost of every reversal, noisy steering error jittering across zero was repeatedly slamming one wheel between full-forward and full-reverse. Fixed, matching the explicit request ("you can brake one motor and move the other to steer"): while `linear >= 0`, each wheel is now clamped to `[0, ...]` instead of allowed negative — a turn slows/stops the inside wheel rather than reversing it. An explicit reverse *linear* command still allows both wheels negative as before; only reverse-as-a-side-effect-of-steering is removed. Only affects the AUTONOMOUS/`mixDifferential` path — MANUAL driving mixes each stick directly and is unaffected (a human pulling a stick back is a real, intentional reverse command, not steering noise).

  Two more layers added in `mission_logic.py`/`mission_manager.py` (not firmware) to reduce oscillation at the *source*, both unit-tested (73/73 passing): a deadzone (`WALL_ERROR_DEADZONE_M`/`HEADING_DEADZONE_RAD`) so lidar-noise-sized wall-distance/heading differences never produce a steering command at all, and a `SteeringRateLimiter` slew-rate limit (`MAX_ANGULAR_SLEW_PER_S`) so even a real, sudden steering demand eases in over ~0.4s rather than snapping — skipped during `hard_stop`, which should stop turning immediately, not ease out of it.

  **Dashboard autonomous-engage — ADDED THEN REVERTED, same day (2026-09-11).** Briefly added a second way to select autonomous from the dashboard (new serial command `M,<autonomous 0|1>\n`, a `/mission/autonomous_request` topic, a toggle button) alongside the physical SwB switch. **Explicitly reverted the same day**: "I want autonomous turn on to be just the switch, nothing from the dashboard." All of it removed from `esp32_firmware.ino` (the `M,` command, `cmd_mode_autonomous` state), `serial_bridge.py` (the topic subscription/forwarding), and `web_dashboard.py` (the button) — autonomous is selected ONLY by SwB again, no other path. The dashboard keeps a **read-only** "Autonomous status" panel showing what the ESP32 actually reports (`autonomous_mode`/`estop_ok` from `/esp32/diagnostics`, parsed there for the first time this session) — display only, no control.
  **Turn aggressiveness raised (2026-09-11).** Explicit request: "I want it to take relatively large steers." `K_LATERAL` 0.6→1.0, `MAX_ANGULAR` 0.8→1.0, `MAX_ANGULAR_SLEW_PER_S` 2.0→3.0 (`mission_logic.py`) — safe to raise together because the deadzones added earlier the same day (`WALL_ERROR_DEADZONE_M`/`HEADING_DEADZONE_RAD`) already filter noise-sized error at the source, so a bigger gain amplifies genuine steering error into a punchier turn rather than reintroducing the jitter those deadzones were built to remove. **Not yet hardware-confirmed** — if turns overshoot/oscillate at the real speed_scale, lower `K_LATERAL` back down before touching the deadzones again.
  **Corner-turn boost added (2026-09-11) — `mission_logic.py`'s `compute_drive()`.** Explicit request: "something guaranteed for the turn." Raw steering strength only came from the left/right distance difference - if that difference is still small while closing in on a wall dead ahead (a square corner, not a gradual bend), the turn could be too weak to clear it before `CAUTION_DISTANCE_M`'s speed cutoff, risking a `hard_stop` nose-first into the wall instead of a completed turn. `speed_scale` (already 1.0 far away, 0.0 at `SAFETY_STOP_M`) is reused as the boost driver: `angular_lateral` gets multiplied by up to `CORNER_TURN_BOOST_MAX+1` (1.5, so up to 2.5x) as the wall gets closer, so steering strength ramps up automatically instead of only slowing down. Safe with no real opening (`angular_lateral` already near zero in that case, and boosting zero stays zero) - only sharpens a turn already being attempted. 3 new tests verifying the exact multiplier math, 76/76 passing. **Not yet hardware-confirmed.**
  **Reverse-from-steering allowed again, with a minimum commitment instead of a ban (2026-09-11) — `motor_mixer.cpp` + `esp32_firmware.ino`.** The "no reverse from steering" fix two entries up was too blunt: the user pointed out the real skid-steer kinematics - stop one track, drive the other, and the vehicle pivots around that stopped track's own centerline at a radius equal to the track separation (now `~1.01m`, the real measurement); only genuine opposite-direction wheel rotation gives a tighter, true zero-radius pivot. Banning reverse from steering entirely removed that tight-pivot capability along with the oscillation it was meant to fix. **Explicit correction**: "you can do reverse but it needs not to be a very small reverse, at least one second of reverse." `mixDifferential()` reverted to a plain stateless mix (no more `[0, ...]` clamp - steering can compute a negative wheel again). The actual fix moved to a new minimum-commitment layer in `applyMotorOutputs()`: once a wheel's *raw* desired direction flips into reverse, `applyReverseCommitment()` forces it to stay reversing for at least `MIN_REVERSE_COMMIT_MS` (1000ms, `config.h`) before a flip back to forward is allowed to take effect, regardless of what a noisy raw signal does meanwhile - measured from when reverse began, not reset on every check. This sits *underneath* the existing `DIRECTION_REVERSAL_BRAKE_MS`/`REVERSAL_RAMP_MS` electrical-settling machinery (unchanged): the commitment layer decides *whether* a flip is allowed to happen at all (rate-limiting real flips to roughly once per second or slower), the electrical machinery still handles *how* each flip that does happen gets executed (brake, hold, ramp).
  **Reverse banned outright in AUTONOMOUS only - third and final correction on this same question, same day (2026-09-11).** Report: "the robot is going backward, not in the parkour way, in autonomous" - the reverse-assisted pivot from the entry directly above was technically correct differential-drive kinematics, but visibly wrong for a vehicle that's supposed to be navigating an obstacle course by only ever advancing and turning. `mixDifferential()` - which is called *exclusively* from the AUTONOMOUS branch of `loop()` (MANUAL mixes each stick directly onto its own track and never calls this function at all) - now unconditionally clamps both wheels to `[0, ...]`, full stop: `out.left = std::max(0.0f, linear - angular)`, same for `out.right`. Because this function is autonomous-only, the clamp scopes itself automatically - **MANUAL driving keeps full reverse capability**, including the `applyReverseCommitment()` 1-second minimum-hold protection from the entry above, completely untouched. Net effect of these three iterations, in order: (1) ban reverse from steering entirely -> (2) allow it back with a 1s minimum commitment, for tight autonomous pivots -> (3) ban it again, but only for autonomous, since the vehicle backing up mid-course was the actual problem being solved, not steering-driven wheel reversal in the abstract. **Not yet flashed or hardware-tested** - firmware-only change, needs a fresh Arduino IDE flash before any of this (including the two entries above it) takes effect on the real board.
  **Nav2 goal source built (2026-09-11) — `teknofest_ws_bridge/nav2_goal_feeder.py`.** Real gap found while preparing to actually launch Nav2: every server in `nav2_hardware.launch.py` comes up correctly, but nothing was ever built to send it a navigation goal, and since this stack deliberately has no persistent map, there is no "click a point on a map" goal source either — without a goal, `bt_navigator` just waits forever and the robot never moves. This new node continuously sends a `NavigateToPose` goal a fixed distance directly ahead of the robot's current position/heading (looked up via tf2 in the `odom` frame at send time, not one static point), re-sending once each goal finishes (succeeded, aborted, or times out) so the robot always has a live goal to plan toward. Makes Nav2 behave as an obstacle-avoiding "always advance" system — a real capability step up from `mission_manager.py`'s simpler wall-following heuristic, since Nav2's local costmap + MPPI controller can route *around* an obstacle instead of just slowing for one dead ahead — but this is explicitly NOT objective-directed navigation to an actual course goal; with no map, it has no notion of "the finish," only "keep advancing, avoid what's ahead." Launched alongside `nav2_hardware.launch.py` via `deploy_updates.sh`'s phase7 (plain `python3`, not an ament-installed executable, matching `mission_manager.py`'s own non-package deployment style). **Entirely untested** — like the rest of the Nav2 hardware stack, has never run against the real robot.
  **`rf2o_laser_odometry` resolved (2026-09-11).** Confirmed missing as a prebuilt package for ROS2 Jazzy (both `apt-cache search` and `apt-cache policy` came back completely empty even after a fresh `apt-get update` — not a network hiccup, genuinely not published as a `.deb` for this distro). Built from source instead: cloned `MAPIRlab/rf2o_laser_odometry` (its `ros2` branch is the default) into `~/teknofest_ws/src/` and `colcon build`'d — succeeded, `install/rf2o_laser_odometry` present. `deploy_updates.sh`'s phase6 checks for this automatically now and reports which path is needed.
  **`robot_description` package deployment automated (2026-09-11).** Never had an automated deploy path before this session — `deploy_updates.sh`'s new phase6 pushes the whole package to `~/teknofest_ws/src/teknofest_description/` and rebuilds it; confirmed working, package builds cleanly on the Pi.
  **Climb power boost + turn-then-advance maze-solving strategy (2026-09-11) — `mission_logic.py`/`mission_manager.py`.** Explicit request: "make sure that the motor values of the vehicle change based on gyro values to make it able pass all parkour stages... in the situation of turns use the strategy changing angular then moving forward a bit changing angular then moving forward a bit with the gyro as a reference. Overall improve our robot as a maze solver robot." Two additions:
  1. **Climb boost.** New `quat_to_pitch_deg()` (same aerospace ZYX extraction `heading_reference.py`'s `quat_to_euler()` uses for yaw, isolated to pitch) computes absolute pitch from the raw IMU quaternion in `mission_manager.py`'s `on_imu()`. `compute_drive()` gained an optional `pitch_deg` parameter (defaults to 0.0, so every existing call/test is unaffected) that boosts `linear` up to `CLIMB_BOOST_MAX` (+100%) as pitch approaches `CLIMB_PITCH_FULL_BOOST_DEG` (15°, not measured) - proactive, from the tilt itself, unlike `StallDetector` which only reacts after forward acceleration has already stopped. **`PITCH_CLIMB_SIGN` is UNCONFIRMED** - the IMU's mount has only ever been validated for yaw (via `heading_reference.py`), never pitch specifically; tilt the vehicle nose-up by hand and confirm `quat_to_pitch_deg()` reports positive before trusting this.
  2. **Turn-then-advance state machine (`TurnAdvanceController`).** Replaces continuous blended steering with two committed phases, matching how a maze-solving robot actually behaves: **ALIGN** (the reactive wall/heading error, `compute_drive()`'s `angular` return value, exceeds `TURN_ALIGN_ENTER`=0.4 - forward motion stops, the vehicle rotates in place using that same error as closed-loop feedback until it drops below `TURN_ALIGN_EXIT`=0.15 - two different thresholds so the state can't chatter at the boundary) and **ADVANCE** (error is small - drive forward at normal speed with only a small heading-hold trim, clamped to `ADVANCE_ANGULAR_TRIM_MAX`=0.15). This satisfies "decrease speed or even stop for a strong turn" for free: ALIGN's forced `linear=0` already is that stop, triggered by the same signal that would otherwise demand a strong turn. It also composes cleanly with the same-day "no reverse in autonomous" rule (`motor_mixer.cpp`): with `linear=0` during ALIGN, `mixDifferential()`'s `[0, ...]` clamp turns a nonzero angular into a genuine pivot (one wheel held at 0, the other driving) automatically - "rotate about yourself" without needing true reverse. During ALIGN the `SteeringRateLimiter` is bypassed entirely (a committed turn should be decisive, not eased in); during ADVANCE it still smooths the small trim correction as before. `CRUISE_LINEAR` stays at 0.5 (already "half of total," matching the explicit request directly) - `climb_boost`/`speed_scale`/stall-boost still modulate it up or down from there depending on conditions, not raising the baseline itself.
  **Real bug found and fixed while writing tests for this**: the ENTER/EXIT thresholds were originally defined in radians (`math.radians(10)`/`math.radians(3)`) under the assumption `compute_drive()`'s `angular` return value was a raw heading-error angle - it is not, it is already a normalized -1..1 command (`clamp(..., -MAX_ANGULAR, MAX_ANGULAR)`, `MAX_ANGULAR=1.0`). Comparing a normalized command against a radian-scaled threshold happened to "work" numerically (both are small numbers) but meant `ADVANCE_ANGULAR_TRIM_MAX`'s clamp was unreachable dead code - any signal large enough to reach it would already have tripped ALIGN first. Renamed to `TURN_ALIGN_ENTER`/`TURN_ALIGN_EXIT` (no more misleading `_RAD` suffix) and rescaled to normalized units (0.4/0.15) so the trim clamp (0.15, deliberately below `TURN_ALIGN_ENTER`) can actually apply. 93/93 tests passing (mission_logic + aim_pipeline combined). **Not yet flashed, deployed, or hardware-tested** - the motor_mixer.cpp/esp32_firmware.ino changes from earlier the same day still need a fresh Arduino IDE flash; the Python side needs a fresh `deploy_updates.sh` push once the Pi is reachable.
  **Reverse-only-if-braked verified, Python side deployed (2026-09-11).** Explicit request: "you can
  use the reverse only if the motor is braked, and don't keep reversing until it meets this condition
  after several time, make sure everything is stable and verify the work to pass the parkour
  logically, then update the orange pi codes." Traced `applyMotorOutputs()`/`applyReverseCommitment()`
  in `esp32_firmware.ino` against ground truth rather than memory: **both constraints were already
  correctly satisfied by existing code, no new mechanism needed.** Reverse always commands max
  magnitude (see the "reverse always full magnitude" entry above), so `p.leftSpeedDac`/`rightSpeedDac`
  is unconditionally above `DAC_OFF_VALUE` on every reverse entry with no exception - meaning the
  `leftFlip`/`rightFlip` check reliably fires on every single reverse entry, including straight from a
  dead stop, and unconditionally routes through the `DIRECTION_REVERSAL_BRAKE_MS` brake+zero-voltage
  hold before any real reverse torque is applied. There is no path that skips it. "Don't keep
  reversing without meeting the condition" is likewise already covered: `MIN_REVERSE_COMMIT_MS`
  (1000ms) prevents a reverse period from ending early, and every re-entry re-triggers the same brake
  gate - no path to repeated ungated reversing exists. Updated a stale code comment on the
  `leftFlip`/`rightFlip` line that pre-dated the reverse-always-max rule and no longer described
  current behavior accurately. 93/93 tests passing (`mission_logic` + `aim_pipeline`), verified both
  on Windows and natively on the Pi (aarch64) after deploy. **Deployed to the Orange Pi**: all of
  today's Python changes (`mission_logic.py`, `mission_manager.py`, `test_mission_logic.py`,
  `nav2_goal_feeder.py`, systemd units) via `deploy_updates.sh` phase1+phase4 - confirmed via md5sum
  match pre- and post-restart, all 9 services active with no duplicate processes, no lingering Nav2
  processes found. **The ESP32 firmware itself is still NOT flashed** - none of tonight's `.ino`/`.cpp`
  changes (brake fix, DAC revert, reversal ramp, reverse-always-max, `applyReverseCommitment`, the
  autonomous-only reverse ban in `mixDifferential`, climb boost's pitch read, turn-then-advance) take
  effect on the real board until a fresh Arduino IDE flash - this has been outstanding across multiple
  rounds of firmware edits now and is the actual blocker before any of tonight's behavior can be
  physically tested.
  **Wall-contact prevention for curves and the sliding door, plus a reverse impact guard
  (2026-09-11) — `mission_logic.py`/`mission_manager.py`/`esp32_firmware.ino`.** Explicit
  request: "make sure that the robot can pass crossing level [curved ways] and sliding door
  without touch the walls, and if he reverse the way make a safety check using the gyro
  values ... to not damage the walls of the parkours, also make sure that robot do the
  parkour stages as documented in the competition." Grounded against
  `docs/robot_project_plan.md` (official stage table) and `docs/run2-autonomy-plan.html`
  (the already-written Run 2 design doc - its "Sliding door" card specifies the exact
  strategy implemented below) and the real behavior-string list in
  `ugv_stage_sign_detector/stage_manager_node.py`'s `DEFAULT_BEHAVIORS` (confirmed
  `"sliding_doors"` is the literal string `ExecuteStage` goals carry for stage 6). Three
  pieces:
  1. **Side-wall contact guard (curved-corridor safety, `compute_drive()`).** Everything
     before this only reacted to a wall dead AHEAD (`SAFETY_STOP_M`/`CAUTION_DISTANCE_M`) -
     through a curve (the two U-turns especially - `robot_project_plan.md` already flags the
     spec's 5m turning-radius requirement against a 3m corridor as a real geometry squeeze)
     the vehicle's FLANK can close on a wall while the forward sector still reads clear.
     New `SIDE_CONTACT_MARGIN_M` (0.65m, a raw lidar-range threshold - accounts for the
     lidar sitting near centerline and the hull's real 0.55m half-width, leaving ~0.10m
     margin once triggered) checked on both side sectors independently of the normal
     proportional centering response: if either side drops below it, `compute_drive()`
     immediately returns `(0.0, ±SIDE_CONTACT_STEER, 'side_contact_risk')` - stop advancing,
     steer hard away from whichever side is critical (0.0 angular if pinched on both sides
     at once, since steering either way then risks closing faster on the other wall).
     `mission_manager.py`'s `drive_loop` treats this mode like `hard_stop` (bypasses
     `TurnAdvanceController`/`SteeringRateLimiter` entirely) since a smoothed/clamped escape
     steer would defeat the point. Bumped three pre-existing centering tests off their
     original 0.3m/0.5m "close wall" test distances (now legitimately inside the new
     margin) up to just above it, preserving their original intent (verify steering-sign
     correctness) without colliding with the new emergency threshold.
  2. **Sliding-door commit logic (Stage 6, new `SlidingDoorController` class) - the
     first stage to get real behavior-specific driving; every other stage is still Phase 1
     baseline corridor-following (see `mission_manager.py`'s own docstring).** Implements
     `run2-autonomy-plan.html`'s documented strategy exactly: stop at `DOOR_STANDOFF_M`
     (1.8m, outside the swept volume) and OBSERVE - count full blocked→clear cycles via
     `/door/state` (door_tracking.py's `DoorTracker`, new subscription in
     `mission_manager.py`) until `MIN_CYCLES_OBSERVED` (3) is reached, matching the doc's
     "watch ≥3 full cycles" - then WAIT_WINDOW until the door reads clear AND its last
     confirmed direction is RIGHT (the doc's committed strategy: hug the right wall, enter
     only after a confirmed right-side exit - a left exit or ambiguous read is refused
     outright) AND the predicted remaining clear window (`period_s/2 - clear_elapsed_s`, a
     simplifying half-period estimate - `period_s` is a full cycle, not a clear/blocked
     split, which `/door/state` doesn't expose separately) exceeds
     `DOOR_TRANSIT_TIME_S * DOOR_TRANSIT_SAFETY_FACTOR` (3.0s × 1.5 - transit time is a
     placeholder pending real DAC-to-m/s speed calibration, same open item as everywhere
     else in this file). Then COMMIT: timed drive at `DOOR_COMMIT_LINEAR` (0.7, faster than
     `CRUISE_LINEAR` since hesitation burns the window) with a right-wall hug
     (`DOOR_RIGHT_HUG_SETPOINT_M`=0.5m, tighter than the normal 0.8m follow setpoint),
     aborting straight back to WAIT_WINDOW (not an error path - a first-class transition,
     per the doc) if the gate re-blocks mid-transit. **This stage's own documented timeout
     fallback is deliberately different from every other stage's** ("hold and keep
     re-observing," not "keep driving straight" - creeping blindly into a moving door on a
     timeout is exactly the collision this exists to prevent), so `DOOR_OBSERVE_TIMEOUT_S`
     (20s) just re-arms the observe window rather than ever inventing forward motion.
     `door_tracking.py`'s `DoorState.as_list()` gained one additive field (`blocked`, index
     6 - the tracker's own hysteresis-debounced state, exact rather than re-derived from
     `blocked_fraction` downstream) - existing consumers reading only indices 0-5
     (`web_dashboard.py`'s `on_door`) are unaffected. `mission_manager.py` dispatches to
     this controller only while `current_behavior == "sliding_doors"` and `compute_drive()`
     isn't already demanding `hard_stop` (a genuine close-range front obstacle always wins);
     resets the controller fresh on every new `sliding_doors` `ExecuteStage` goal and on
     every full mission stop (not on a momentary scan/heading blip - those already fully
     stop the vehicle anyway, no need to discard in-progress door observation for one).
     12 new tests (state transitions, cycle-counting, direction/window gating, commit-abort,
     right-hug steering sign, reset) - 109/109 passing overall.
  3. **Reverse impact guard (`esp32_firmware.ino`, gyro-based, per explicit
     request).** Backing up is blind on this vehicle - no rear-facing sensor exists - so the
     only way to know it has actually backed into a wall is from what the impact itself does
     to the chassis. New `checkReverseImpactGuard()`: while either wheel is reversing (only
     then - gated to bound the extra I2C read to exactly the windows where it matters,
     since unconditional per-loop BNO055 reads were the exact root cause of a real gimbal-
     stepper-starvation bug fixed 2026-09-07), reads gyro magnitude and compares it against
     the previous iteration's sample; a jump bigger than `REVERSE_IMPACT_GYRO_DELTA`
     (config.h) is treated as a collision, since the vehicle's own motors cannot produce an
     abrupt single-iteration step this way (`DIRECTION_REVERSAL_BRAKE_MS` holds zero voltage
     through every flip and `REVERSAL_RAMP_MS` eases speed in over half a second - any
     genuine self-induced turning while reversing builds up gradually, never as one sudden
     jump). On detection: both wheels forced to `DAC_OFF_VALUE` immediately, the electrical
     brake relay forced engaged for `REVERSE_IMPACT_BRAKE_HOLD_MS` (500ms), and the reverse
     commitment released early - safety overrides `MIN_REVERSE_COMMIT_MS`'s "keep reversing
     at least a second" rule outright, since continuing to satisfy a minimum-duration timer
     is never correct once a collision is actually detected. The gyro-magnitude baseline is
     dropped (not carried over) the instant reversing stops, so the next reversing spell
     always seeds fresh rather than diffing against data that could be seconds stale.
     **Known limitation, scoped out of tonight's change**: this catches a sudden IMPACT
     (an abrupt kick), not a sustained PUSH against something already contacted (e.g. the
     operator keeps holding the stick back after the first hit already stopped the
     vehicle) - a reverse-specific stall detector mirroring the existing forward
     `StallDetector` would be the natural follow-up if that gap matters in practice.
     **UNVALIDATED PLACEHOLDER threshold** - whether `VECTOR_GYROSCOPE` reads rad/s or
     deg/s on this specific board has never been independently confirmed (the wire-protocol
     header comment claims rad/s; the Adafruit library's common default is deg/s - not
     resolved). `REVERSE_IMPACT_GYRO_DELTA`=80.0 is a deliberately high starting point
     (unlikely to false-trigger on normal driving) - log `s_lastGyroMag` on real hardware
     during normal reversing (safe noise floor) and a deliberate light bump test (real
     trigger magnitude) before trusting this, per the working agreement. **Firmware-only -
     needs a fresh Arduino IDE flash before any of this (or anything else from tonight)
     takes effect on the real board**, same outstanding item as every other firmware change
     this session.

  **A rear-lidar proximity warning (dashboard-visible, non-blocking) was deliberately NOT
  added this pass** - the gyro impact guard above directly satisfies the explicit request
  and works in any mode without new control-authority questions. A Pi-side auto-brake
  override of MANUAL stick input during reverse was considered and explicitly NOT built:
  manual driving (Run 1) never routes through `/cmd_vel` at all currently (the ESP32 only
  honors it in AUTONOMOUS), so a lidar-based auto-brake there would need a brand-new
  override channel that can countermand the human operator mid-drive - a genuinely
  different, more consequential wiring decision than an impact-only safety net, and Run 1
  is explicitly human-piloted and the reward-ranking completion gate, so overriding
  operator authority there deserves an explicit decision, not a default silently chosen.
  Revisit if the impact guard alone proves insufficient in practice.

  **Orange Pi reachable over Tailscale when the LAN alias times out (2026-09-11).** The documented SSH
  alias `orangepi` (`192.168.137.239`) went completely unreachable mid-session (`Connection timed
  out`) while the Pi was actually up and running - confirmed by reaching it instead over its Tailscale
  IP, `<pi-tailscale-ip>` (the Tailscale hostname is not `orangepi` - same physical machine,
  confirmed via aarch64/rockchip kernel + all 9 `teknofest-*` services present). `deploy_updates.sh`'s
  `PI=orangepi` was changed to `PI="${PI:-orangepi}"` so it can be overridden per-invocation
  (`PI=jolian@<pi-tailscale-ip> bash deploy_updates.sh ...`) without touching the default. Tailscale
  itself also flaps independently of the LAN - `tailscale status` is the fastest ground-truth check
  (an "offline, last seen Nm ago" annotation on the Pi's row means genuinely down, not a local routing
  problem) before assuming either the LAN or Tailscale path is the real outage.
- ~~Three-camera requirement~~ — **Decided:** 3× USB cameras (front drive, rear drive, aim — mounted on the pan/tilt servo), replacing the single Pi Camera v3 plan. Being purchased; not yet physically connected or integrated into the vision pipeline (Stage 1/6 still not started). Must be powered via a powered/self-powered USB hub, not bus-powered off the Orange Pi (see Open Decisions power budget item).
- **RC/manual control path (Run 1)** — SBUS input to the ESP32 is now wired into firmware (`esp32_firmware/`), but the operator-side video downlink for FPV driving is still undesigned.
- **URDF chassis/sensor dimensions** (2026-08-26) — `robot_description/urdf/teknofest.urdf.xacro` created with tank tracks modeled as static boxes (fixed joints, skid-steer kinematics — no continuous/wheel joints needed for tracks), lidar/IMU/front/rear cameras as fixed mounts, and the aim camera on real revolute pan/tilt joints matching the aiming gimbal's pan/tilt axes (stepper-driven now, not PCA9685 — see the gimbal stepper integration entry above; the URDF's joint limits still reference an old servo-range comment that should be revisited). **Overall footprint now a real measurement (2026-09-11, user-provided): 149cm long x 110cm wide, symmetric about center.** `chassis_length`/`track_separation`/`track_width` in the xacro and the `footprint` polygon in both `nav2_params.yaml` and `nav2_params_hardware.yaml` were updated to match (half-extents 0.745m x 0.55m). `chassis_width` (hull-only, narrower than the full track width) and everything else not covered by this one length/width measurement — `chassis_height`, sensor mount offsets like `lidar_xyz`'s 0.10m forward offset, mass values — are still unmeasured guesses. Given the new length is much closer to `mission_logic.py`'s `SAFETY_STOP_M=0.8` comment (which already assumed "vehicle is 1.45m long," oddly closer to reality than the URDF's own old 0.50m guess was), that stop distance is probably in the right ballpark, but the real clearance it leaves at the front bumper depends on `lidar_xyz`'s unconfirmed mount offset — validate at low speed before trusting it.
- **Camera USB2 limit is CONFIRMED NOT SOFTWARE-FIXABLE** (2026-09-03). Measured on all three
  cameras: each reaches 30 fps alone (C50 54 fps), but **any pair leaves the second at 0 fps**.
  Ruled out in testing: MJPG compression (`cv2.CAP_PROP_FOURCC`), and the `uvcvideo`
  `quirks=128` FIX_BANDWIDTH path — `uvcvideo` is **built into this kernel**, not a module, so it
  can't be modprobed; writing `128` to `/sys/module/uvcvideo/parameters/quirks` and forcing
  re-enumeration via the `authorized` toggle did take effect but changed nothing. The limit is UVC
  isochronous bandwidth reservation on one 480 Mbps bus.
  **The fix is physical**: all three sit on one Terminus hub (bus 007); the Genesys/GL3523 hub
  (bus 001/002) is completely free. Move at least one camera onto that separate host controller.
  Until then `camera_manager_node.py` owns all three and switches between them — which suits the
  mission plan, where the aim camera is only needed at a standstill.
- **Camera USB hub is USB2, not USB3** (found 2026-08-25) — `lsusb -t` shows both cameras enumerated under a USB **2.0** hub (`1a86:8095` QinHeng, 480M link), while the Orange Pi's onboard USB **3.0** SuperSpeed root hub (5000M, `GL3523`) sits idle with nothing attached. Starting the front (C30) and aim (C50) `usb_cam` nodes at the same time reliably fails the second node with `Unable to start stream` — reproduced even at 320×240@20fps, so it's not a raw-bitrate problem; Linux's UVC isochronous bandwidth reservation is sized off each camera's *maximum* supported mode (2560×1440), and two such reservations don't fit in a USB2 hub's ~480Mbps budget regardless of the resolution actually requested. **Needs a physical fix**: move the camera hub (or at least one camera) onto a USB 3.0 port/hub — do not attempt to work around this in software (lower resolutions don't help, confirmed). Not yet resolved; front-only vision development can proceed in the meantime.

## Comm topics (serial bridge path — `serial_bridge.py` on the Orange Pi)
| Topic | Direction | Purpose | Status |
|---|---|---|---|
| `/cmd_vel` | Pi → ESP32 | velocity command (`geometry_msgs/Twist`, linear.x + angular.z only) | writes confirmed, no error |
| `/esp32/imu` | ESP32 → Pi | BNO055 orientation quaternion, gyro, accel (`sensor_msgs/Imu`) | **verified, ~18.6 Hz** |
| `/esp32/diagnostics` | ESP32 → Pi | loop Hz, SBUS link health, armed state, autonomous-mode flag, e-stop status (`std_msgs/Float32MultiArray`: [loop_hz, sbus_ok, armed, autonomous_mode, estop_ok]) | **[loop_hz, sbus_ok, armed] verified**; `autonomous_mode`/`estop_ok` appended 2026-09-07 (purely additive); now consumed by `heading_reference.py` (see below), **not yet exercised on real hardware** |
| `/esp32/odom` | ESP32 → Pi | fused heading + velocity, into EKF | **not implemented** in the current protocol/bridge — was in the original micro-ROS plan, needs adding back if the EKF requires it |
| `/esp32/aim_cmd` | Pi → ESP32 | autonomous aim+fire command, `A,<pan>,<tilt>,<fire>\n` (pan/tilt normalized -1..1, fire 0/1), only honored when SwB=autonomous AND SwC=aim position — has full authority over both gimbal direction and the laser relay in that state | ESP32 side implemented 2026-08-31; `serial_bridge.py` forwards it (done 2026-08-31); published by `aim_tracker.py`, revised 2026-09-08 to route through `target_detection.py`'s `TargetFilter`+`PID`+`pixels_to_degrees` instead of a bare per-pixel gain (see the Vision strategy note below) — **this row was stale until now, incorrectly saying no publisher existed** |
| `/mission/active` | Pi-internal (`heading_reference.py` → `mission_manager.py`) | `std_msgs/Bool` — armed && autonomous_mode && estop_ok, latched from `/esp32/diagnostics` and failed to False if diagnostics go stale (>1s) | code-complete 2026-09-08, **not yet hardware-tested** |
| `/mission/heading` | Pi-internal (`heading_reference.py` → `mission_manager.py`) | `std_msgs/Float32MultiArray`: [roll_rel, pitch_rel, yaw_rel (rad), zeroed 0/1] — IMU orientation relative to whatever it read at the last manual→autonomous rising edge; this is the concrete mechanism behind "reset the gyro to 000" (a software reference offset, not an actual sensor re-zero — see the file's docstring for why) | code-complete 2026-09-08, **not yet hardware-tested** |
| `/mission/state`, `/mission/current_behavior` | `mission_manager.py` → Pi (dashboard, future) | `std_msgs/String` — coarse drive-loop state (`idle`/`stopped`/`running:<mode>`) and the currently-active stage's behavior label, if any | code-complete 2026-09-08, **not yet hardware-tested**; no dashboard card reads these yet |
| `/mission/execute_stage` (action) | `ugv_stage_sign_detector`'s `stage_action_dispatcher_node` → `mission_manager.py` | `ugv_stage_interfaces/action/ExecuteStage` — one goal per validated stage-sign transition (`stage_id`, `behavior` string); `mission_manager.py` accepts every goal and just tracks it, since **no stage currently gets its own driving behavior** (Phase 1 — see the file's docstring) | code-complete 2026-09-08, **not yet hardware-tested**; action name is the dispatcher's own default, verified against its launch files (no override present) |
| `/mission/reset_stage_sequence` | `heading_reference.py` → `stage_manager_node` (`ugv_stage_sign_detector` package) | `std_msgs/Empty` — published once on every fresh manual→autonomous rising edge; tells `stage_manager_node` to drop back to its initial stage so a second attempt (disarm, reposition, re-arm) isn't rejected as out-of-order against wherever the previous attempt left off. Deliberately does NOT publish a `StageTransition` (a reset is not a sign-triggered behavior change). Fixes a real gap found 2026-09-08: `stage_manager_node` shipped with no reset mechanism at all. | code-complete 2026-09-08, unit-tested (`test_mission.py`: `test_reset_returns_to_initial_stage_for_a_second_attempt`, `test_reset_honors_a_non_zero_initial_stage`, 48/48 passing), **not yet hardware-tested** |

Wire format is documented in the header comment of `esp32_firmware/arduino_sketch/esp32_firmware/esp32_firmware.ino`.

~~**Needs a matching update on the Pi side**~~ — **done (2026-08-31)**: `serial_bridge.py`'s docstring and `/esp32/aim_cmd` handling are current with the DAC-based drivetrain (`leftSpeedDac`/`rightSpeedDac`/`leftReverse`/`rightReverse`); it also now forwards `/esp32/aim_cmd` → the `A,<pan>,<tilt>,<fire>` serial command for autonomous aim+fire (published by `teknofest_ws_bridge/aim_tracker.py`).

## Immediate next task (rewritten 2026-09-03)
The perception stack is deployed and running; the vehicle has never moved under its own power.
In order:
1. **Bind the FS-A8S receiver.** `/esp32/diagnostics` still reports `sbus_ok=0, armed=0`, so no motor
   path has been exercised end to end and the DAC speed calibration (`DAC_MIN_VALUE`/`DAC_MAX_VALUE`)
   is still uncalibrated. Do this with wheels off the ground.
2. **Measure the physical constants that currently poison every range estimate**: front camera height
   and pitch (for `cone_terrain.py`), and the URDF chassis/sensor dimensions.
3. **Move one camera to the free Genesys hub** (bus 001/002) so two can stream at once.
4. **Recapture segmentation frames on real course-like terrain** and retrain — the current checkpoint
   learned an indoor room.
5. Remove the ESP32 firmware's temporary `G,panSpeedScale=...` debug line once gimbal accel is confirmed.
6. Still open from before: decide whether `/esp32/odom` is needed before Stage 5 (EKF).
7. **New (2026-09-08): `heading_reference.py` + `mission_manager.py`** — Phase 1 autonomous mission
   (gyro-zero on manual→autonomous engagement, reactive lidar corridor-following, `ExecuteStage`
   action server tracking the real `ugv_stage_sign_detector` pipeline's stage transitions). Code-
   complete, **zero hardware testing** — needs, in order: (a) items 1-2 above done first (armed/SBUS
   working, DAC calibrated) since this publishes `/cmd_vel` the same way manual driving does; (b) an
   explicit decision on the `stage_manager_node` reset gap (it has no way to reset `current_stage`
   back to 0 between separate autonomous engagements within one process lifetime — see the note
   Claude raised with the user on 2026-09-08); (c) real tuning of every gain/distance in
   `mission_manager.py` (`SAFETY_STOP_M`, `CAUTION_DISTANCE_M`, `K_LATERAL`, `K_HEADING`,
   `CRUISE_LINEAR` — all placeholders) against the real vehicle at low `speed_scale`; (d) confirming
   how far around the vehicle the front-mounted lidar can actually see, since the corridor controller
   degrades to heading-hold-only wherever it can't see a wall and that degradation has never been
   observed on the real sensor.

## Superseded next task
Get the Orange Pi back online (was unreachable as of 2026-08-31 — DHCP IP change or genuinely powered off, check `~/.ssh/config`) and, once the front camera is reachable: (1) validate `sign_detector.py`'s CNN against an actual printed sign under real lighting — print `vision_prototyping/generate_laser_target.py`'s sibling or a real stage sign mockup and test detection range/angle; (2) validate `path_segmentation_node.py`'s classical fallback against real `/front/image_raw` frames and tune `ground_plane_heuristic.py`'s `COLOR_DIST_THRESHOLD`; (3) once a few dozen real frames exist, run them through the heuristic to produce pseudo-labels and try training `vision_models/path_segmentation/train.py`'s CNN for the first time. Separately, still pending from before: get the FS-A8S/SBUS receiver physically wired and bound so `sbus_ok`/`armed` in `/esp32/diagnostics` can be exercised for real, then measure real speed-DAC calibration (`esp32_firmware/arduino_sketch/esp32_firmware/config.h`'s `DAC_MIN_VALUE`/`DAC_MAX_VALUE` placeholders) with wheels off the ground before trusting any `/cmd_vel` → motor motion. Also decide whether `/esp32/odom` needs implementing before Stage 5 (EKF).
