"""
Face-tracking pan/tilt demo. Runs on the Windows laptop, NOT the Orange Pi.

What this proves: camera -> face detection (largest bounding box) -> serial
command -> ESP32 differential-gimbal steppers reacting in real time. The aim
camera is NOT physically mounted on the gimbal yet, so this is a decoupled
functional test of the control loop, not a true closed-loop "camera aimed at
what it sees" system - don't read the pan/tilt motion as physically correct
until the camera actually rides the gimbal.

Requires the venv in this folder (plain global Python's opencv-python 4.13.0
cannot read frames from the LUMINOUS C50 on this machine - both DSHOW and
MSMF backends failed; opencv-python==4.10.0.84 in this venv is confirmed
working). Run with:

    .venv\\Scripts\\python.exe face_track_aim_demo.py

Talks to the ESP32 over the same serial line protocol as
esp32_firmware/arduino_sketch/differential_gimbal_test/differential_gimbal_test.ino
(the 'm <pan> <tilt>' relative-move command) - flash that sketch first.
"""
import time

import cv2
import serial

# ============================================================
# Confirmed working on this machine - see conversation history if either
# breaks after a Windows update or camera/ESP32 gets unplugged:
#   - Camera: LUMINOUS C50 = DirectShow device index 0, CAP_DSHOW backend
#   - ESP32: COM4 (Silicon Labs CP210x)
# ============================================================
CAMERA_INDEX = 0
SERIAL_PORT = "COM4"
BAUD = 115200

CAPTURE_WIDTH = 1920
CAPTURE_HEIGHT = 1080
PROCESS_WIDTH = 640  # downscale for detection/display speed; Haar cascade is slow at full res

# Pixel-error-to-steps gain. Tune by watching how twitchy/sluggish the
# response is - this has not been calibrated to real angles, just a
# reasonable starting point for a demo.
GAIN_PAN = 0.15
GAIN_TILT = 0.15
DEADZONE_PX = 20          # ignore error smaller than this (avoids jitter when face is ~centered)
MAX_STEPS_PER_UPDATE = 25  # smaller+more-frequent moves read as smoother than big infrequent jumps

# Flip these to -1 if the gimbal moves the wrong way relative to the face -
# there's no way to know the correct sign without watching the real hardware.
PAN_SIGN = -1
TILT_SIGN = -1

CONTROL_LOOP_HZ = 25  # raised alongside the smaller MAX_STEPS_PER_UPDATE clamp above

# Exponential moving average on the detected face position - Haar cascade
# bounding boxes jitter a few pixels frame to frame even for a still face,
# and that noise was going straight into the motor commands. 0 = no
# smoothing (raw, jittery), closer to 1 = heavier smoothing (laggier).
SMOOTHING_ALPHA = 0.35


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    for _ in range(10):  # warm-up, first frames are often black/stale
        cap.read()
        time.sleep(0.03)
    return cap


def open_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2.0)  # ESP32 auto-resets when the serial port opens (DTR toggle) - wait for boot
    ser.reset_input_buffer()
    ser.write(b"e\n")  # enable drivers
    ser.write(b"s 500\n")  # faster per-step timing (default 800us) - less visibly "steppy" motion
    return ser


def largest_face(faces):
    if len(faces) == 0:
        return None
    return max(faces, key=lambda f: f[2] * f[3])  # f = (x, y, w, h), area = w*h


def main():
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    cap = open_camera()
    ser = open_serial()
    print("Enabled drivers. 'q' in the video window to quit.")

    min_period = 1.0 / CONTROL_LOOP_HZ
    last_update = 0.0
    filtered_ex = None
    filtered_ey = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue

            scale = PROCESS_WIDTH / frame.shape[1]
            small = cv2.resize(frame, (PROCESS_WIDTH, int(frame.shape[0] * scale)))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
            face = largest_face(faces)

            h, w = small.shape[:2]
            cx, cy = w // 2, h // 2
            cv2.drawMarker(small, (cx, cy), (255, 0, 0), cv2.MARKER_CROSS, 20, 1)

            now = time.time()
            if face is not None:
                fx, fy, fw, fh = face
                face_cx, face_cy = fx + fw // 2, fy + fh // 2
                cv2.rectangle(small, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
                cv2.drawMarker(small, (face_cx, face_cy), (0, 255, 0), cv2.MARKER_CROSS, 15, 1)

                ex = face_cx - cx
                ey = face_cy - cy

                if filtered_ex is None:
                    filtered_ex, filtered_ey = ex, ey
                else:
                    filtered_ex = SMOOTHING_ALPHA * ex + (1 - SMOOTHING_ALPHA) * filtered_ex
                    filtered_ey = SMOOTHING_ALPHA * ey + (1 - SMOOTHING_ALPHA) * filtered_ey

                if now - last_update >= min_period:
                    last_update = now
                    cmd_ex = 0 if abs(filtered_ex) < DEADZONE_PX else filtered_ex
                    cmd_ey = 0 if abs(filtered_ey) < DEADZONE_PX else filtered_ey
                    pan_steps = int(max(-MAX_STEPS_PER_UPDATE, min(MAX_STEPS_PER_UPDATE, PAN_SIGN * GAIN_PAN * cmd_ex)))
                    tilt_steps = int(max(-MAX_STEPS_PER_UPDATE, min(MAX_STEPS_PER_UPDATE, TILT_SIGN * GAIN_TILT * cmd_ey)))
                    if pan_steps != 0 or tilt_steps != 0:
                        ser.write(f"m {pan_steps} {tilt_steps}\n".encode())
                    cv2.putText(small, f"err=({filtered_ex:.0f},{filtered_ey:.0f}) cmd=({pan_steps},{tilt_steps})",
                                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            else:
                cv2.putText(small, "no face", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            cv2.imshow("face_track_aim_demo", small)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        ser.write(b"x\n")  # disable drivers before exiting
        ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
