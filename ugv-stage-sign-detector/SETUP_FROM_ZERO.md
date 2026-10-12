# Setup From Zero

This is the **start-here guide** for a brand-new Windows 11 laptop and, later,
the Raspberry Pi 5 competition computer.

The project has two operating modes:

| Computer | Purpose | Camera | ROS 2 required? |
| --- | --- | --- | --- |
| Windows 11 laptop | Develop, test videos, use a USB/built-in webcam, and calibrate the detector | Laptop or USB webcam | No |
| Raspberry Pi 5 (8 GB) | Run the real robot system and publish confirmed stages to ROS 2 | Raspberry Pi Camera Module 3 | Yes |

Do not try to connect Camera Module 3 directly to the Windows laptop. It is a
CSI camera intended for the Raspberry Pi. Develop on Windows with the included
test video or a normal webcam.

## 1. Keep this exact folder hierarchy

The safest arrangement is to use the supplied ZIP and choose **Extract All**.
The ZIP already contains the correct hierarchy. Do not take the Python files
out of their package folder, and do not move either ROS package.

```text
ugv-stage-sign-detector/
├── SETUP_FROM_ZERO.md                 This guide
├── README.md                          Project overview and commands
├── FULL_CODE_GUIDE.md                 Detailed code and data-flow explanation
├── ML_TRAINING_GUIDE.md               Hybrid dataset/training/deployment guide
├── requirements.txt                   Windows Python dependencies
├── requirements-ml.txt                Laptop-only ML training dependencies
├── LICENSE
├── .gitignore
│
├── test_output/                       Test data; not program source
│   ├── digit_templates/               sign_1.png ... sign_12.png
│   ├── printable_signs/               Full printable example signs
│   ├── test_scenes/                   Synthetic detector test images
│   ├── sequence_test.mp4              Included end-to-end test video
│   ├── detection_preview.png
│   └── sign_montage.png
│
└── ros2_ws/                           ROS 2 workspace
    └── src/                           ROS packages must be directly in src
        ├── ugv_stage_interfaces/      Package 1: custom messages/action
        │   ├── msg/
        │   │   ├── StageSign.msg
        │   │   └── StageTransition.msg
        │   ├── action/
        │   │   └── ExecuteStage.action
        │   ├── CMakeLists.txt
        │   └── package.xml
        │
        └── ugv_stage_sign_detector/   Package 2: vision and ROS nodes
            ├── ugv_stage_sign_detector/
            │   ├── __init__.py
            │   ├── config.py
            │   ├── detector.py
            │   ├── classification.py
            │   ├── templates.py
            │   ├── temporal.py
            │   ├── mission.py
            │   ├── synthetic.py
            │   ├── cli.py
            │   ├── calibration.py
            │   ├── generate_signs.py
            │   ├── ml_dataset.py
            │   ├── capture_dataset.py
            │   ├── ml_model.py
            │   ├── ml_train.py
            │   ├── ros_node.py
            │   ├── stage_manager_node.py
            │   ├── action_dispatcher_node.py
            │   └── mock_action_server.py
            ├── config/
            │   ├── detector.yaml
            │   └── detector_ml.yaml
            ├── models/                 Trained ONNX model is written here
            ├── launch/
            │   ├── sign_detector.launch.py
            │   ├── pi_camera_and_detector.launch.py
            │   └── action_demo.launch.py
            ├── test/
            │   ├── test_config.py
            │   ├── test_detector.py
            │   ├── test_temporal.py
            │   └── test_mission.py
            ├── resource/
            │   └── ugv_stage_sign_detector
            ├── setup.py
            ├── setup.cfg
            └── package.xml
```

The two directories below must be siblings:

```text
ros2_ws/src/ugv_stage_interfaces
ros2_ws/src/ugv_stage_sign_detector
```

ROS builds `ugv_stage_interfaces` first because the detector package depends on
its messages. Do not put one package inside the other.

Temporary directories such as `.venv`, `.pytest_cache`, `build`, `install`,
`log`, `__pycache__`, or `*.egg-info` are generated automatically. They do not
belong in a clean copy and should never be moved into `ros2_ws/src`.

## 2. Windows 11: install everything from zero

Use Windows first. This proves that the CV code and its tests work before ROS
or the robot hardware is involved.

### 2.1 Install Python

1. Download the **64-bit Python 3.10 Windows installer** from:
   <https://www.python.org/downloads/windows/>
2. Run it.
3. Enable **Add python.exe to PATH** when the installer offers that option.
4. Keep the Python Launcher (`py`) enabled.
5. Finish the installation.

Open **PowerShell** from the Start menu and check it:

```powershell
py --version
py -3.10 --version
```

Both commands should print a Python version. The commands below intentionally
use Python 3.10, matching this project's development environment.

### 2.2 Extract and locate the project

1. Create `C:\UGV` in File Explorer.
2. Put `UGV_Stage_Sign_Detector_v0.3.0.zip` there.
3. Right-click the ZIP and choose **Extract All**.
4. After extraction, verify that this exact file exists:

```text
C:\UGV\ugv-stage-sign-detector\requirements.txt
```

If you received all the files separately, manually recreate the hierarchy in
section 1. Using the ZIP is much less error-prone.

### 2.3 Create an isolated Python environment

Run these commands in PowerShell:

```powershell
cd C:\UGV\ugv-stage-sign-detector
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
.\.venv\Scripts\python.exe -m pip install -e .\ros2_ws\src\ugv_stage_sign_detector
```

The `.venv` directory keeps this project's Python packages separate from the
rest of the laptop. The commands intentionally use its full Python path, so
PowerShell script-execution policy cannot prevent activation.

### 2.4 Verify the installation

```powershell
.\.venv\Scripts\python.exe -c "import cv2; import ugv_stage_sign_detector; print('OpenCV:', cv2.__version__); print('Detector:', ugv_stage_sign_detector.__version__)"
```

Expected detector version:

```text
Detector: 0.3.0
```

### 2.5 Run all automated tests

```powershell
.\.venv\Scripts\python.exe -m pytest .\ros2_ws\src\ugv_stage_sign_detector\test -q
```

The final line must report that all tests passed. The exact count can increase
as regression cases are added; any red `FAILED` line is not normal.

### 2.6 Run the included end-to-end video test

```powershell
.\.venv\Scripts\ugv-sign-detect.exe --source .\test_output\sequence_test.mp4 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector.yaml --templates .\test_output\digit_templates --headless
```

The terminal should print confirmed JSON events for sign 4 and then sign 5.
This test is headless, so it does not open a video window.

### 2.7 Generate Windows Arial Black templates

The competition specification uses Arial Black. Generate a Windows-specific
set without overwriting the included test data:

```powershell
.\.venv\Scripts\ugv-generate-signs.exe .\windows_assets --font "C:\Windows\Fonts\ariblk.ttf"
```

This creates:

```text
windows_assets/digit_templates
windows_assets/printable_signs
windows_assets/test_scenes
```

Keep `windows_assets/digit_templates` with the project when it is copied to the
Raspberry Pi.

### 2.8 Run with the laptop webcam

```powershell
.\.venv\Scripts\ugv-sign-detect.exe --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector.yaml --templates .\windows_assets\digit_templates
```

Hold a printed test sign in front of the webcam. Press `Q` or Escape to stop.

If camera 0 does not open:

1. Close the Windows Camera app and any program already using the webcam.
2. In Windows Settings, allow desktop apps to access the camera.
3. Try `--source 1` instead of `--source 0`.

### 2.9 Calibrate HSV thresholds for your lighting

```powershell
.\.venv\Scripts\ugv-sign-calibrate.exe --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector.yaml --templates .\windows_assets\digit_templates --save .\calibrated_detector.yaml
```

Move the HSV sliders until the mask contains the red ring but excludes most
other objects. Press `S` to save and `Q` or Escape to quit. Then test the saved
configuration:

```powershell
.\.venv\Scripts\ugv-sign-detect.exe --source 0 --config .\calibrated_detector.yaml --templates .\windows_assets\digit_templates
```

`calibrated_detector.yaml` is machine/lighting-specific. Do not treat laptop
calibration as final competition calibration.

### 2.10 Train the hybrid classifier

The template configuration above remains useful for calibration and regression
tests, but it is not the recommended real classifier. Continue with
[ML_TRAINING_GUIDE.md](ML_TRAINING_GUIDE.md) to install the laptop-only ML
requirements, generate/capture data, train the model, and test
`config\detector_ml.yaml` before moving the project to the Raspberry Pi.

## 3. Raspberry Pi 5: install the competition runtime from zero

Recommended target:

- Raspberry Pi 5, 8 GB, with an active cooler
- Raspberry Pi Camera Module 3 and the correct Pi 5 camera cable
- 64-bit Ubuntu Server 24.04 LTS
- ROS 2 Jazzy

Use Ubuntu 24.04 because ROS 2 Jazzy has supported ARM64 binary packages for
that operating-system release. The commands in this section run on the Pi, not
in Windows PowerShell.

### 3.1 Flash Ubuntu to a microSD card

On the Windows laptop:

1. Install Raspberry Pi Imager from <https://www.raspberrypi.com/software/>.
2. Insert a good microSD card.
3. In Imager choose Raspberry Pi 5.
4. Choose **Ubuntu Server 24.04 LTS (64-bit)**.
5. Preconfigure a username, password, hostname, Wi-Fi, locale, and SSH.
6. Write and verify the card.

Shut down and disconnect power before attaching Camera Module 3. Insert the
cable in the correct orientation and use a Pi 5-compatible camera cable. Then
insert the microSD card and boot the Pi.

### 3.2 Connect to the Pi

From Windows PowerShell, replace `robot` and `ugv-pi` with the username and
hostname chosen in Imager:

```powershell
ssh robot@ugv-pi.local
```

If Windows reports that `ssh` is unknown, enable **OpenSSH Client** under
Settings > System > Optional features, or connect a keyboard and display to the
Pi temporarily.

All commands from here through section 3.9 are entered in the Pi's Linux shell.

### 3.3 Update Ubuntu and enable required repositories

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

Ubuntu on Raspberry Pi may omit the update and backport suites that ROS needs.
Check the current setting:

```bash
grep '^Suites:' /etc/apt/sources.list.d/ubuntu.sources
```

It must include all three names:

```text
Suites: noble noble-updates noble-backports
```

If it does not, edit the file:

```bash
sudo nano /etc/apt/sources.list.d/ubuntu.sources
```

Change only the `Suites:` line to the value above. In nano, press `Ctrl+O`,
Enter, then `Ctrl+X`. Apply the update and reboot:

```bash
sudo apt clean
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

Reconnect with SSH after the Pi restarts.

### 3.4 Install ROS 2 Jazzy

Add the official ROS repository:

```bash
sudo apt update
sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt full-upgrade -y
```

Install the headless ROS runtime and development tools used to build this
workspace:

```bash
sudo apt install -y ros-jazzy-ros-base ros-dev-tools python3-opencv python3-pil python3-yaml
```

Initialize ROS dependency management:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` says it was already initialized, continue normally.
Make ROS available in every new shell:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
ros2 --help
```

### 3.5 Copy the project from Windows to the Pi

Copy the original ZIP so that the Windows `.venv` is not transferred to Linux.
In a new **Windows PowerShell** window, run:

```powershell
scp C:\UGV\UGV_Stage_Sign_Detector_v0.3.0.zip robot@ugv-pi.local:/home/robot/
```

Change both occurrences of `robot` if your Pi username is different. Return to
the Pi's SSH terminal and extract it:

```bash
sudo apt install -y unzip
cd ~
unzip UGV_Stage_Sign_Detector_v0.3.0.zip
```

After following `ML_TRAINING_GUIDE.md`, copy the trained ONNX file if training
was completed after the project ZIP was transferred:

```powershell
scp C:\UGV\ugv-stage-sign-detector\ros2_ws\src\ugv_stage_sign_detector\models\stage_sign_classifier.onnx robot@ugv-pi.local:/home/robot/ugv-stage-sign-detector/ros2_ws/src/ugv_stage_sign_detector/models/
```

Return to the Pi's SSH terminal and verify the package layout:

```bash
test -f ~/ugv-stage-sign-detector/ros2_ws/src/ugv_stage_interfaces/package.xml && echo interfaces-ok
test -f ~/ugv-stage-sign-detector/ros2_ws/src/ugv_stage_sign_detector/package.xml && echo detector-ok
```

Both `interfaces-ok` and `detector-ok` must appear.

### 3.6 Build Camera Module 3 support and `camera_ros`

Camera Module 3 uses the Sony IMX708 sensor. The `camera_ros` project currently
documents that full IMX708 support on Ubuntu requires Raspberry Pi's libcamera
fork, so build both projects in a separate camera workspace.

Install libcamera's required build tools plus the `cam` test application's
dependencies:

```bash
sudo apt install -y g++ meson ninja-build pkg-config libyaml-dev python3-yaml python3-ply python3-jinja2 libgnutls28-dev openssl libevent-dev libdrm-dev libjpeg-dev libsdl2-dev libtiff-dev python3-colcon-meson
```

Create and build the camera workspace:

```bash
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src
git clone https://github.com/raspberrypi/libcamera.git
git clone https://github.com/christianrauch/camera_ros.git
cd ~/camera_ws
source /opt/ros/jazzy/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy --skip-keys=libcamera
colcon build --event-handlers=console_direct+
source ~/camera_ws/install/setup.bash
```

Make the camera overlay available in future shells, after base ROS but before
the UGV workspace:

```bash
echo 'source ~/camera_ws/install/setup.bash' >> ~/.bashrc
```

Test whether libcamera sees the sensor:

```bash
cam -l
```

The output should contain `imx708` or `imx708_wide`. If it says no cameras are
available, stop here and recheck the powered-off cable connection before
debugging the vision code.

### 3.7 Build the UGV ROS workspace

```bash
cd ~/ugv-stage-sign-detector/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Verify that ROS sees both packages:

```bash
ros2 pkg prefix ugv_stage_interfaces
ros2 pkg prefix ugv_stage_sign_detector
```

Make the UGV overlay available in every future terminal:

```bash
echo 'source ~/ugv-stage-sign-detector/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

The `.bashrc` source order should now be base ROS, `camera_ws`, then the UGV
workspace. That order is important.

### 3.8 Test the camera alone

Start Camera Module 3 at 1280x720:

```bash
ros2 run camera_ros camera_node --ros-args -p width:=1280 -p height:=720
```

Leave it running. Open a second SSH terminal and check the topic:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw
```

You should see `/camera/image_raw` and a non-zero frame rate. Stop the camera
with `Ctrl+C` before the next section.

### 3.9 Run the complete perception-to-ROS-action pipeline

Verify that the trained model exists inside the source package before building:

```bash
test -f ~/ugv-stage-sign-detector/ros2_ws/src/ugv_stage_sign_detector/models/stage_sign_classifier.onnx && echo model-ok
```

If `model-ok` does not appear, train/copy the model and rebuild the UGV
workspace. The hybrid node intentionally refuses to start without it.

Run the hybrid detector and mission nodes:

```bash
ros2 launch ugv_stage_sign_detector sign_detector.launch.py \
  config_file:=$(ros2 pkg prefix ugv_stage_sign_detector)/share/ugv_stage_sign_detector/config/detector_ml.yaml \
  image_topic:=/camera/image_raw
```

This launch starts:

1. the sign detector subscriber;
2. temporal confirmation;
3. the sequential stage manager; and
4. the ROS Action dispatcher.

It does **not** start the camera, so open a second terminal and run:

```bash
ros2 run camera_ros camera_node --ros-args -p width:=1280 -p height:=720
```

Until your teammate's real `/mission/execute_stage` Action server is ready,
open a third terminal and start the safe mock server:

```bash
ros2 run ugv_stage_sign_detector mock_stage_action_server
```

Open a fourth terminal to observe the outputs:

```bash
ros2 topic echo /perception/stage_sign/confirmed
```

Other useful inspections, run one at a time, are:

```bash
ros2 topic echo /mission/stage_transition
ros2 topic echo /mission/current_stage
ros2 action info /mission/execute_stage
```

When a sign is detected in enough consecutive frames, the flow is:

```text
Camera image -> red-ring CV -> ONNX number -> confirmed sign -> stage manager -> ExecuteStage goal
```

The detector never sends GPIO, steering, brake, motor, or laser commands
directly. Your teammate's Action server owns those robot-control decisions.

### 3.10 Simpler all-in-one launch

After the camera workspace is working, this launch starts `camera_ros` and all
included UGV nodes in one terminal:

```bash
ros2 launch ugv_stage_sign_detector pi_camera_and_detector.launch.py
```

The explicit multi-terminal method in section 3.9 is better for first-time
debugging because camera failures and detector failures remain separate.

## 4. What must be calibrated before the competition

The supplied thresholds and generated signs are a software starting point, not
a final field guarantee. Before connecting the Action server to real actuators:

1. Record real Camera Module 3 footage of every official sign.
2. Include the actual competition lighting, distance, motion blur, tilt, and
   red barriers in the recordings.
3. Calibrate the HSV/geometry portion using those recordings.
4. Train and validate the ONNX classifier on separate recording sessions.
5. Confirm the official mapping and order for signs 1-12 with the committee.
6. Test the complete ROS pipeline with the mock server.
7. Let the controls teammate replace the mock with the real Action server only
   after an emergency-stop and safe bench-test procedure exists.

## 5. Fast troubleshooting table

| Symptom | Most likely fix |
| --- | --- |
| `py` is not recognized on Windows | Re-run the Python installer and enable PATH/Python Launcher |
| `No module named cv2` | Repeat the pip commands using `.venv\Scripts\python.exe` |
| Webcam will not open | Close other camera apps, allow camera privacy access, try source 1 |
| Tests cannot find files | Run them from `C:\UGV\ugv-stage-sign-detector` |
| `ros2` is not found | Run `source /opt/ros/jazzy/setup.bash` |
| UGV ROS package is not found | Source `ros2_ws/install/setup.bash` after building |
| `no cameras available` | Power off, reseat the cable, then verify `cam -l` |
| `/camera/image_raw` is absent | Start `camera_ros` and verify it did not exit with an error |
| Detector sees red barriers as signs | Tighten geometry/HSV thresholds using real recorded footage |
| ONNX model does not exist | Train it on Windows, copy it into the package `models` directory, then rebuild ROS |
| Dispatcher says Action server unavailable | Start the mock server or teammate's `/mission/execute_stage` server |

## 6. Normal command order after installation

On Windows:

```powershell
cd C:\UGV\ugv-stage-sign-detector
.\.venv\Scripts\ugv-sign-detect.exe --source 0 --config .\ros2_ws\src\ugv_stage_sign_detector\config\detector_ml.yaml
```

On the Raspberry Pi:

```bash
source ~/.bashrc
ros2 launch ugv_stage_sign_detector pi_camera_and_detector.launch.py
```
