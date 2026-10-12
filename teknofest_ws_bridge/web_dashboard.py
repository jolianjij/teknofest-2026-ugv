#!/usr/bin/env python3
"""
Single-page telemetry dashboard for the whole vehicle, served off the Pi.

One ROS 2 node subscribes to everything and serves it over plain HTTP:
    /                     the dashboard
    /api/state            JSON snapshot (IMU, SBUS, lidar, mission, door...)
    /stream/front.mjpg    front camera as MJPEG
    /stream/debug.mjpg    sign-detector debug view as MJPEG

Deliberately dependency-free beyond what the Pi already has (rclpy, cv2,
numpy, cv_bridge) and with no CDN assets, because this network throttles
downloads unpredictably - a dashboard that needs to fetch a charting
library is a dashboard that fails in the pit. Lidar and attitude are drawn
by hand on a canvas.

Every subscription is optional: topics that are not running simply show as
stale rather than preventing the dashboard from starting. That matters
because the point of this page is diagnosing which half of the stack is
down.

Run on the Pi:  python3 web_dashboard.py --ros-args -p port:=8080
Then open      http://<pi-address>:8080/
"""
import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String, UInt8
from cv_bridge import CvBridge

try:  # the sign-detector interfaces are a separate colcon package
    from ugv_stage_interfaces.msg import StageSign, StageTransition
    HAVE_STAGE_MSGS = True
except ImportError:
    HAVE_STAGE_MSGS = False

LIDAR_BEAMS = 360      # downsampled before serialising; the browser cannot use 2000 points
ENCODE_MIN_INTERVAL_S = 0.1   # cap MJPEG encoding at ~10 fps per stream (default)
# Front is the one stream driving is done off of - explicitly asked to be
# real-time/no-lag (2026-09-10). Raised to ~20fps, matching the camera's
# own real capture rate (~23fps) closer than the shared 10fps default.
# Rear/aim/debug/segmentation stay at the conservative default: this is the
# same encode-CPU tradeoff the 10fps cap below was originally added for,
# now applied to only one stream instead of all five so the extra load is
# bounded - if capture rates (checked via /camera/status) ever regress
# after this, lower ENCODE_MIN_INTERVAL_FRONT_S back down rather than the
# shared default.
ENCODE_MIN_INTERVAL_FRONT_S = 0.05
STREAM_MAX_WIDTH = 960        # downscale before encoding; panels are far smaller
STALE_AFTER_S = 2.0

# ~/teknofest_ws/ is where this script itself lives once deployed (see
# CLAUDE.md's repo layout) - mission_recorder.py is deployed alongside it.
HERE = Path(__file__).resolve().parent
RECORDINGS_DIR = Path.home() / "teknofest_ws" / "recordings"
RECORDER_STATUS_POLL_S = 1.0


def quaternion_to_euler_deg(x, y, z, w):
    """Roll/pitch/yaw in degrees. Pitch is clamped rather than allowed to
    produce NaN at gimbal lock, which would blank the whole attitude panel."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class DashboardState:
    """Latest value per topic, with the time it arrived so the page can grey
    out anything that has gone stale instead of showing a frozen number."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = {}
        self.jpegs = {}

    def put(self, key, value):
        with self.lock:
            self.data[key] = {"value": value, "t": time.time()}

    def snapshot(self):
        now = time.time()
        with self.lock:
            return {
                key: {
                    "value": entry["value"],
                    "age": round(now - entry["t"], 2),
                    "stale": (now - entry["t"]) > STALE_AFTER_S,
                }
                for key, entry in self.data.items()
            }

    def set_jpeg(self, which, buf):
        with self.lock:
            self.jpegs[which] = buf

    def get_jpeg(self, which):
        with self.lock:
            return self.jpegs.get(which)


STATE = DashboardState()
# The HTTP handler needs to publish camera-switch requests, but handlers are
# constructed per-request by the server, so the node is reached through this
# holder rather than being threaded through the handler constructor.
NODE_REF = {"node": None}


class TelemetryNode(Node):
    def __init__(self):
        super().__init__("web_dashboard")
        self.declare_parameter("port", 8080)
        self.bridge = CvBridge()
        self._encode_times = {}
        self._recorder_proc = None
        self._bag_proc = None
        self._recording_dir = None
        # start_recording/stop_recording run on an HTTP handler thread (see
        # Handler.do_GET), not the ROS executor thread that
        # poll_recording_status runs on - both touch the same _recorder_proc/
        # _recording_dir attributes, so this guards against a real race (e.g.
        # two quick clicks on Record spawning two subprocesses).
        self._recording_lock = threading.Lock()
        self.create_timer(RECORDER_STATUS_POLL_S, self.poll_recording_status)

        self.create_subscription(Image, "/front/image_raw", self.on_front, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/esp32/imu", self.on_imu, 10)
        self.create_subscription(Float32MultiArray, "/esp32/diagnostics", self.on_diag, 10)
        self.create_subscription(Float32MultiArray, "/esp32/sbus_channels", self.on_sbus, 10)
        self.create_subscription(Float32MultiArray, "/esp32/motor_pwm", self.on_motor, 10)
        self.create_subscription(Float32MultiArray, "/door/state", self.on_door, 10)
        self.create_subscription(Float32MultiArray, "/front/ground_obstacles", self.on_obstacles, 10)
        self.create_subscription(UInt8, "/mission/current_stage", self.on_stage, 10)
        # For the lidar panel's heading/steering overlay (2026-09-11) - the
        # robot's zeroed-heading drift (yaw_rel, from heading_reference.py)
        # and mission_manager.py's live steering decision (/cmd_vel, only
        # actually driven while in AUTONOMOUS - manual driving never touches
        # this topic, see esp32_firmware.ino's MANUAL branch).
        self.create_subscription(Float32MultiArray, "/mission/heading", self.on_mission_heading, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/camera/status", self.on_cam_status, 5)
        self.create_subscription(Image, "/aim/image_raw", self.on_aim, qos_profile_sensor_data)
        self.create_subscription(Image, "/rear/image_raw", self.on_rear, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/front/segmentation_overlay", self.on_seg, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/aim/debug_image", self.on_aim_debug, qos_profile_sensor_data
        )
        # Cameras now stream all three simultaneously (camera_manager_node.py
        # rewritten 2026-09-10 - each is on its own independent USB
        # controller). This publisher is a per-camera RESET trigger now, not
        # a switch - "front"/"rear"/"aim"/"all" forces that worker to
        # release and reopen its device immediately, for when a camera looks
        # stuck rather than cleanly disconnected.
        self.reset_pub = self.create_publisher(String, "/camera/reset", 5)
        self.tuning_pub = self.create_publisher(Float32MultiArray, "/tuning/drive", 5)
        self.create_subscription(
            Float32MultiArray, "/tuning/drive_state", self.on_tuning_state, 5
        )

        if HAVE_STAGE_MSGS:
            self.create_subscription(
                StageSign, "/perception/stage_sign/observation", self.on_sign_obs, 10
            )
            self.create_subscription(
                StageSign, "/perception/stage_sign/confirmed", self.on_sign_conf, 10
            )
            self.create_subscription(
                StageTransition, "/mission/stage_transition", self.on_transition, 10
            )
            self.create_subscription(
                Image, "/perception/stage_sign/debug_image", self.on_debug_img,
                qos_profile_sensor_data,
            )

        self.get_logger().info(
            f"dashboard on http://0.0.0.0:{self.get_parameter('port').value}/ "
            f"(stage msgs {'available' if HAVE_STAGE_MSGS else 'MISSING'})"
        )

    # ---- image topics -> JPEG for MJPEG streaming ----
    def _encode(self, msg, which):
        """Rate-limited JPEG encoding.

        Encoding every frame of five image topics was costing more CPU than
        the perception it was displaying, and starved the camera capture loop
        (front camera fell to 5 fps). A browser cannot use more than ~10 fps
        of MJPEG anyway, so drop anything arriving faster.
        """
        now = time.time()
        last = self._encode_times.get(which, 0.0)
        min_interval = ENCODE_MIN_INTERVAL_FRONT_S if which == "front" else ENCODE_MIN_INTERVAL_S
        if now - last < min_interval:
            return
        self._encode_times[which] = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        # Big frames (the aim camera runs at 2560x1440) are pointless to ship
        # at full size to a browser panel a few hundred pixels wide.
        if frame.shape[1] > STREAM_MAX_WIDTH:
            scale = STREAM_MAX_WIDTH / float(frame.shape[1])
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            STATE.set_jpeg(which, buf.tobytes())
            STATE.put(f"{which}_image", {"w": frame.shape[1], "h": frame.shape[0]})

    def on_front(self, msg):
        self._encode(msg, "front")

    def on_aim(self, msg):
        self._encode(msg, "aim")

    def on_rear(self, msg):
        self._encode(msg, "rear")

    def on_seg(self, msg):
        self._encode(msg, "segmentation")

    def on_aim_debug(self, msg):
        self._encode(msg, "aimdebug")

    def on_debug_img(self, msg):
        self._encode(msg, "debug")

    def on_cam_status(self, msg):
        try:
            STATE.put("camera_status", json.loads(msg.data))
        except ValueError:
            pass

    def reset_camera(self, name):
        self.reset_pub.publish(String(data=name))

    # ---- mission recording (front/rear/aim cameras + telemetry bag) ----
    def start_recording(self):
        with self._recording_lock:
            return self._start_recording_locked()

    def _start_recording_locked(self):
        if self._recorder_proc is not None or self._bag_proc is not None:
            return {"ok": False, "error": "already recording"}

        session_dir = RECORDINGS_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir.mkdir(parents=True, exist_ok=True)

        recorder_script = HERE / "mission_recorder.py"
        try:
            self._recorder_proc = subprocess.Popen(
                ["python3", str(recorder_script), str(session_dir)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as e:
            self.get_logger().error(f"failed to start mission_recorder.py: {e}")
            return {"ok": False, "error": str(e)}

        # Telemetry alongside the video - the standard tool for this, not
        # custom code. Topics chosen to cover what a post-run review of a
        # mission attempt would actually need; failing to record any one of
        # these (e.g. a topic not currently running) does not stop the bag
        # from recording the rest.
        bag_topics = [
            "/esp32/imu", "/esp32/diagnostics", "/scan", "/cmd_vel",
            "/mission/active", "/mission/heading", "/mission/state",
            "/mission/current_behavior", "/mission/current_stage",
            "/perception/stage_sign/confirmed", "/mission/stage_transition",
            "/door/state", "/tuning/drive_state", "/esp32/aim_cmd",
        ]
        try:
            self._bag_proc = subprocess.Popen(
                ["ros2", "bag", "record", "-o", str(session_dir / "telemetry_bag"), *bag_topics],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as e:
            self.get_logger().warning(f"telemetry bag failed to start (video recording continues): {e}")
            self._bag_proc = None

        self._recording_dir = session_dir
        STATE.put("recording", {"active": True, "dir": str(session_dir), "cameras": {}})
        self.get_logger().info(f"recording started: {session_dir}")
        return {"ok": True, "dir": str(session_dir)}

    def stop_recording(self):
        with self._recording_lock:
            return self._stop_recording_locked()

    def _stop_recording_locked(self):
        if self._recorder_proc is None and self._bag_proc is None:
            return {"ok": False, "error": "not recording"}

        for proc, name in ((self._recorder_proc, "mission_recorder"), (self._bag_proc, "ros2 bag")):
            if proc is None:
                continue
            try:
                proc.send_signal(signal.SIGINT)  # let each finalize its own files cleanly
                proc.wait(timeout=10)
            except Exception as e:
                self.get_logger().warning(f"{name} did not exit cleanly, killing: {e}")
                proc.kill()

        session_dir = self._recording_dir
        self._recorder_proc = None
        self._bag_proc = None
        self._recording_dir = None
        STATE.put("recording", {"active": False, "dir": str(session_dir) if session_dir else None, "cameras": {}})
        self.get_logger().info(f"recording stopped: {session_dir}")
        return {"ok": True, "dir": str(session_dir) if session_dir else None}

    def poll_recording_status(self):
        """Reads mission_recorder.py's own status file (per-camera state) so
        the dashboard can show which cameras are actually contributing
        footage right now, not just whether recording is nominally on."""
        with self._recording_lock:
            recording_dir = self._recording_dir
            active = self._recorder_proc is not None
        if recording_dir is None:
            return
        status_path = recording_dir / "recording_status.json"
        try:
            data = json.loads(status_path.read_text())
        except (OSError, ValueError):
            return
        STATE.put("recording", {
            "active": active,
            "dir": str(recording_dir),
            "cameras": data.get("cameras", {}),
        })

    def on_tuning_state(self, msg):
        d = list(msg.data)
        if len(d) >= 3:
            STATE.put("tuning", {
                "speed_scale": round(d[0], 3),
                "max_linear": round(d[1], 3),
                "max_angular": round(d[2], 3),
            })

    def set_tuning(self, speed_scale=-1.0, max_linear=-1.0, max_angular=-1.0):
        """Negative means 'leave unchanged' - see serial_bridge.tuning_callback."""
        msg = Float32MultiArray()
        msg.data = [float(speed_scale), float(max_linear), float(max_angular)]
        self.tuning_pub.publish(msg)

    # ---- data topics ----
    def on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        # Non-finite returns mean "no echo"; send them as 0 so the client can
        # skip them without needing to understand inf/NaN in JSON.
        ranges = np.where(np.isfinite(ranges), ranges, 0.0)
        if ranges.size > LIDAR_BEAMS:
            idx = np.linspace(0, ranges.size - 1, LIDAR_BEAMS).astype(int)
            ranges = ranges[idx]
            increment = msg.angle_increment * (msg.ranges.__len__() / LIDAR_BEAMS)
        else:
            increment = msg.angle_increment
        STATE.put("scan", {
            "angle_min": round(float(msg.angle_min), 5),
            "angle_increment": round(float(increment), 6),
            "range_max": round(float(msg.range_max), 2),
            "ranges": [round(float(r), 2) for r in ranges],
        })

    def on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        STATE.put("imu", {
            "roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 2),
            "quat": [round(q.x, 4), round(q.y, 4), round(q.z, 4), round(q.w, 4)],
            "gyro": [round(msg.angular_velocity.x, 3), round(msg.angular_velocity.y, 3),
                     round(msg.angular_velocity.z, 3)],
            "accel": [round(msg.linear_acceleration.x, 2), round(msg.linear_acceleration.y, 2),
                      round(msg.linear_acceleration.z, 2)],
        })

    def on_diag(self, msg):
        d = list(msg.data)
        STATE.put("diagnostics", {
            "loop_hz": round(d[0], 1) if len(d) > 0 else None,
            "sbus_ok": bool(d[1]) if len(d) > 1 else None,
            "armed": bool(d[2]) if len(d) > 2 else None,
            # autonomous_mode/estop_ok were always in the wire format (see
            # esp32_firmware.ino's D, line) but never parsed here until
            # 2026-09-11 - needed now to confirm the new dashboard
            # autonomous-engage toggle actually took effect on the ESP32
            # (SwB can still override it) rather than just trusting the
            # request was sent.
            "autonomous_mode": bool(d[3]) if len(d) > 3 else None,
            "estop_ok": bool(d[4]) if len(d) > 4 else None,
        })

    def on_sbus(self, msg):
        STATE.put("sbus", [round(v, 1) for v in msg.data])

    def on_motor(self, msg):
        d = list(msg.data)
        STATE.put("motor", {
            "left_dac": round(d[0], 1) if len(d) > 0 else None,
            "right_dac": round(d[1], 1) if len(d) > 1 else None,
            "left_reverse": bool(d[2]) if len(d) > 2 else None,
            "right_reverse": bool(d[3]) if len(d) > 3 else None,
        })

    def on_door(self, msg):
        d = list(msg.data)
        if len(d) < 6:
            return
        STATE.put("door", {
            "direction": d[0], "centroid": round(d[1], 3), "velocity": round(d[2], 1),
            "blocked_fraction": round(d[3], 3), "period_s": round(d[4], 2),
            "clear_elapsed_s": round(d[5], 2),
        })

    def on_obstacles(self, msg):
        d = list(msg.data)
        STATE.put("obstacles", [
            {"range_m": round(d[i], 2), "bearing_deg": round(d[i + 1], 1)}
            for i in range(0, len(d) - 2, 3)
        ])

    def on_stage(self, msg):
        STATE.put("current_stage", int(msg.data))

    def on_mission_heading(self, msg):
        d = list(msg.data)
        if len(d) < 4:
            return
        STATE.put("mission_heading", {"yaw_rel": d[2], "zeroed": d[3] > 0.5})

    def on_cmd_vel(self, msg: Twist):
        STATE.put("cmd_vel", {"linear": msg.linear.x, "angular": msg.angular.z})

    def on_sign_obs(self, msg):
        STATE.put("sign_observation", {
            "sign_id": int(msg.sign_id), "confidence": round(float(msg.confidence), 3),
        })

    def on_sign_conf(self, msg):
        STATE.put("sign_confirmed", {
            "sign_id": int(msg.sign_id), "confidence": round(float(msg.confidence), 3),
            "at": time.strftime("%H:%M:%S"),
        })

    def on_transition(self, msg):
        STATE.put("stage_transition", {
            "previous": int(msg.previous_stage), "current": int(msg.current_stage),
            "behavior": str(msg.behavior), "at": time.strftime("%H:%M:%S"),
        })


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass  # a request log line per frame would drown the console

    def do_GET(self):
        if self.path.startswith("/api/state"):
            body = json.dumps(STATE.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/tune"):
            from urllib.parse import parse_qs, urlparse
            q = parse_qs(urlparse(self.path).query)

            def get(name):
                try:
                    return float(q.get(name, ["-1"])[0])
                except ValueError:
                    return -1.0

            if NODE_REF["node"] is not None:
                NODE_REF["node"].set_tuning(
                    get("speed_scale"), get("max_linear"), get("max_angular")
                )
            body = json.dumps({"ok": True}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/camera/reset"):
            name = self.path.split("cam=")[-1].split("&")[0] if "cam=" in self.path else ""
            ok = name in ("front", "aim", "rear", "all")
            if ok and NODE_REF["node"] is not None:
                NODE_REF["node"].reset_camera(name)
            body = json.dumps({"ok": ok, "reset": name}).encode()
            self.send_response(200 if ok else 400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/record/start"):
            result = NODE_REF["node"].start_recording() if NODE_REF["node"] is not None else {"ok": False, "error": "node not ready"}
            body = json.dumps(result).encode()
            self.send_response(200 if result.get("ok") else 400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/record/stop"):
            result = NODE_REF["node"].stop_recording() if NODE_REF["node"] is not None else {"ok": False, "error": "node not ready"}
            body = json.dumps(result).encode()
            self.send_response(200 if result.get("ok") else 400)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/stream/"):
            which = self.path.rsplit("/", 1)[-1].split(".")[0]
            if which not in ("front", "aim", "rear", "debug", "active",
                             "segmentation", "aimdebug"):
                which = "active"
            self.stream_mjpeg(which)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def stream_mjpeg(self, which):
        """MJPEG stream for one named camera/view - front/rear/aim now all
        stream simultaneously (2026-09-10), so the page opens one connection
        per camera it actually wants to show, not a single switchable one.
        'active' is kept only as a harmless fallback for any caller that
        doesn't pass a specific name."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_sent = None
        try:
            while True:
                name = which
                if name == "active":
                    status = STATE.data.get("camera_status", {}).get("value")
                    name = status.get("active", "front") if status else "front"
                buf = STATE.get_jpeg(name)
                if buf is not None and buf is not last_sent:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode())
                    self.wfile.write(buf)
                    self.wfile.write(b"\r\n")
                    last_sent = buf
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # browser navigated away or switched cameras


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UGV Telemetry</title>
<style>
:root{
  --bg:#0f1319; --panel:#171d26; --panel2:#1e2631; --line:#2a323e;
  --ink:#e4e8ee; --muted:#95a2b3; --accent:#e5a748; --ok:#5fb587; --bad:#e2695a;
  --mono:ui-monospace,"IBM Plex Mono",Menlo,Consolas,monospace;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
  font:14px/1.5 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:14px 20px;
  border-bottom:1px solid var(--line);background:var(--panel)}
h1{font-size:16px;margin:0;letter-spacing:.02em;font-weight:600}
.dot{width:9px;height:9px;border-radius:50%;background:var(--bad)}
.dot.live{background:var(--ok)}
#conn{font-family:var(--mono);font-size:12px;color:var(--muted)}
.grid{display:grid;gap:14px;padding:14px;
  grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;
  padding:14px;min-width:0}
.card h2{font-size:11px;letter-spacing:.12em;text-transform:uppercase;
  color:var(--muted);margin:0 0 10px;font-weight:600;font-family:var(--mono);
  display:flex;justify-content:space-between;align-items:center}
.age{font-size:10px;color:var(--muted)}
.age.stale{color:var(--bad)}
img.cam{width:100%;border-radius:6px;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
canvas{width:100%;display:block;background:#0b0e13;border-radius:6px}
table{width:100%;border-collapse:collapse;font-family:var(--mono);font-size:12.5px}
td{padding:3px 0;vertical-align:top}
td:first-child{color:var(--muted)}
td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.chip{display:inline-block;padding:2px 9px;border-radius:100px;font-family:var(--mono);
  font-size:11px;background:var(--panel2);color:var(--muted)}
.chip.on{background:rgba(95,181,135,.16);color:var(--ok)}
.chip.off{background:rgba(226,105,90,.16);color:var(--bad)}
.bar{height:16px;background:var(--panel2);border-radius:3px;position:relative;overflow:hidden}
.bar>i{position:absolute;top:0;bottom:0;background:var(--accent);opacity:.85}
.sbus-row{display:grid;grid-template-columns:42px 1fr 52px;gap:8px;align-items:center;
  margin-bottom:5px;font-family:var(--mono);font-size:11px;color:var(--muted)}
.sbus-row b{color:var(--ink);text-align:right;font-weight:500;font-variant-numeric:tabular-nums}
.big{font-family:var(--mono);font-size:26px;font-variant-numeric:tabular-nums}
.sub{color:var(--muted);font-size:12px}
.wide{grid-column:1/-1}
.cams{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(240px,1fr))}
.camcell{min-width:0}
.camhead{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;
  font-family:var(--mono);font-size:10.5px;letter-spacing:.08em;color:var(--muted)}
.camnote{font-size:11.5px;color:var(--muted);margin:-4px 0 10px}
.camstate{font-family:var(--mono);font-size:10px}
.camstate.ok{color:var(--ok)} .camstate.warn{color:var(--accent)} .camstate.bad{color:var(--bad)}
button.pick{font-family:var(--mono);font-size:10px;letter-spacing:.06em;cursor:pointer;
  background:var(--panel2);color:var(--muted);border:1px solid var(--line);
  border-radius:100px;padding:2px 10px}
button.pick:hover{color:var(--ink);border-color:var(--accent)}
button.pick.active{background:rgba(229,167,72,.16);color:var(--accent);border-color:var(--accent)}
button.pick:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.tune-row{display:grid;grid-template-columns:82px 1fr 44px;gap:10px;align-items:center;
  margin-bottom:8px;font-family:var(--mono);font-size:11px;color:var(--muted)}
.tune-row b{color:var(--ink);text-align:right;font-weight:500;font-variant-numeric:tabular-nums}
.tune-row input[type=range]{width:100%;accent-color:var(--accent)}
.sys{display:grid;gap:8px;grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
.sysitem{display:flex;align-items:center;gap:8px;background:var(--panel2);
  border:1px solid var(--line);border-radius:6px;padding:8px 10px;min-width:0}
.sysitem .led{width:8px;height:8px;border-radius:50%;flex:0 0 auto;background:var(--muted)}
.sysitem .led.ok{background:var(--ok)} .sysitem .led.bad{background:var(--bad)}
.sysitem .led.warn{background:var(--accent)}
.sysitem .nm{font-size:12px;flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.sysitem .vl{font-family:var(--mono);font-size:11px;color:var(--muted);white-space:nowrap}
</style></head><body>
<header>
  <span class="dot" id="dot"></span>
  <h1>UGV Telemetry</h1>
  <span id="conn">connecting…</span>
</header>
<div class="grid">

  <div class="card wide">
    <h2>Cameras</h2>
    <div class="camnote" id="camNote">all three stream simultaneously - independent USB controllers, no more switching</div>
    <div style="display:flex;gap:6px;align-items:center;margin:-4px 0 10px;flex-wrap:wrap">
      <span style="font-family:var(--mono);font-size:10.5px;color:var(--muted);letter-spacing:.06em">
        solo (driving from far / high-lag link - stops the other two so all the bandwidth goes to one):</span>
      <button class="pick solo-view active" data-solo="all">all three</button>
      <button class="pick solo-view" data-solo="front">front only</button>
      <button class="pick solo-view" data-solo="rear">rear only</button>
      <button class="pick solo-view" data-solo="aim">aim only</button>
    </div>
    <div class="cams">
      <div class="camcell" id="camCell_front">
        <div class="camhead"><span>FRONT</span><span class="age" id="a_front"></span>
          <span class="camstate" id="camState_front"></span>
          <button class="pick reset-cam" data-cam="front">reset</button></div>
        <img class="cam" id="camImg_front" src="/stream/front.mjpg" alt="front camera">
      </div>
      <div class="camcell" id="camCell_rear">
        <div class="camhead"><span>REAR</span><span class="age" id="a_rear"></span>
          <span class="camstate" id="camState_rear"></span>
          <button class="pick reset-cam" data-cam="rear">reset</button></div>
        <img class="cam" id="camImg_rear" src="/stream/rear.mjpg" alt="rear camera">
      </div>
      <div class="camcell" id="camCell_aim">
        <div class="camhead"><span>AIM / TILT (C50)</span><span class="age" id="a_aim"></span>
          <span class="camstate" id="camState_aim"></span>
          <button class="pick reset-cam" data-cam="aim">reset</button></div>
        <img class="cam" id="camImg_aim" src="/stream/aim.mjpg" alt="aim camera">
      </div>
      <div class="camcell">
        <div class="camhead">
          <span>ANALYSIS</span>
          <span class="age" id="a_debug"></span>
        </div>
        <div style="display:flex;gap:6px;margin-bottom:6px;flex-wrap:wrap">
          <button class="pick view active" data-view="debug">signs</button>
          <button class="pick view" data-view="segmentation">segmentation</button>
          <button class="pick view" data-view="aimdebug">laser targeting</button>
        </div>
        <img class="cam" id="analysisView" src="/stream/debug.mjpg" alt="analysis view">
        <div class="camnote" id="analysisNote"></div>
      </div>
    </div>
  </div>

  <div class="card wide">
    <h2>Recording</h2>
    <div class="camnote">Records front + rear + aim cameras (720p, independent resolution from
      the live previews above) plus a telemetry bag (IMU, diagnostics, lidar, mission state)
      into <code>~/teknofest_ws/recordings/&lt;timestamp&gt;/</code>. Opens the camera devices
      directly, same as the live previews - each camera now has its own dedicated USB controller
      so all three record simultaneously without contention; if one drops out its row below will
      show "unavailable" and the others keep recording.</div>
    <div style="display:flex;gap:8px;align-items:center;margin:10px 0">
      <button class="pick" id="recBtn" style="background:#3a1e1e;border-color:#6b3030">record</button>
      <span id="recStatus" class="sub">not recording</span>
    </div>
    <table id="recTbl"></table>
  </div>

  <div class="card wide">
    <h2>System state</h2>
    <div id="sysstate" class="sys"></div>
  </div>

  <div class="card">
    <h2>Lidar <span class="age" id="a_scan"></span></h2>
    <canvas id="lidar" width="440" height="440"></canvas>
    <div class="camnote">Robot-frame view (front is always up, not a world map).
      <span style="color:#6fd3ff">Cyan</span> = heading drift since autonomous last engaged.
      <span style="color:#5fb587">Green</span>/<span style="color:#e2695a">red</span> = current
      steering command (forward/reverse) - the reactive corridor-follower's live decision, not a
      planned path (no path planner is running yet).</div>
  </div>

  <div class="card">
    <h2>Attitude <span class="age" id="a_imu"></span></h2>
    <canvas id="att" width="440" height="220"></canvas>
    <table id="imuTbl"></table>
  </div>

  <div class="card">
    <h2>Mission <span class="age" id="a_stage"></span></h2>
    <div class="big" id="stageNum">—</div>
    <div class="sub" id="stageName">no stage</div>
    <table id="missionTbl" style="margin-top:10px"></table>
  </div>

  <div class="card">
    <h2>Autonomous status</h2>
    <div class="camnote">Read-only. Autonomous is selected ONLY by the physical SwB switch -
      no dashboard control, by explicit request.</div>
    <div id="autoChips"></div>
  </div>

  <div class="card">
    <h2>ESP32 <span class="age" id="a_diag"></span></h2>
    <div style="margin-bottom:10px" id="espChips"></div>
    <table id="espTbl"></table>
  </div>

  <div class="card">
    <h2>SBUS channels <span class="age" id="a_sbus"></span></h2>
    <div id="sbus"><span class="sub">no SBUS data</span></div>
  </div>

  <div class="card">
    <h2>Door / obstacles <span class="age" id="a_door"></span></h2>
    <table id="doorTbl"></table>
  </div>

  <div class="card">
    <h2>Drive tuning <span class="age" id="a_tune"></span></h2>
    <div class="camnote">Scales <code>/cmd_vel</code> before it reaches the motors.
      Can only slow the vehicle, never speed it up. Start low.</div>
    <div class="tune-row">
      <span>speed scale</span>
      <input type="range" id="t_scale" min="0" max="100" value="25">
      <b id="v_scale">0.25</b>
    </div>
    <div class="tune-row">
      <span>max linear</span>
      <input type="range" id="t_lin" min="0" max="100" value="100">
      <b id="v_lin">1.00</b>
    </div>
    <div class="tune-row">
      <span>max angular</span>
      <input type="range" id="t_ang" min="0" max="100" value="100">
      <b id="v_ang">1.00</b>
    </div>
    <div style="display:flex;gap:8px;margin-top:10px;flex-wrap:wrap">
      <button class="pick" id="t_apply">apply</button>
      <button class="pick" id="t_stop">set 0 (hold)</button>
    </div>
    <div class="camnote" id="tuneEcho" style="margin-top:8px">—</div>
  </div>

</div>
<script>
const STAGES={1:"fording & shower",2:"aggregate road",3:"side slope",4:"vertical obstacle",
 5:"traffic cones",6:"sliding doors",7:"bumped road",8:"incline + 2s stop",
 9:"fire zone",10:"decline + 2s stop",11:"acceleration start",12:"acceleration end"};

// Streaming topics go red when they stop arriving. Latched/event topics
// (mission stage, transitions) only publish on change, so ageing them the
// same way would flash a false "NO DATA" alarm mid-run - show elapsed time
// since the event instead, with no stale styling.
function age(id,e,latched){const el=document.getElementById(id);if(!el)return;
  if(!e){el.textContent=latched?"—":"no data";el.className="age"+(latched?"":" stale");return;}
  if(latched){el.textContent=e.age<1?"just now":Math.round(e.age)+"s ago";el.className="age";return;}
  el.textContent=e.age.toFixed(1)+"s";el.className="age"+(e.stale?" stale":"");}
function rows(t,pairs){document.getElementById(t).innerHTML=
  pairs.map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join("");}

// arrow() and the two overlays below share the exact same "increasing angle
// = left" render convention already established by the scan points'
// px_=cx-sin(a)*..., py_=cy-cos(a)*... math (a=0 is straight up/"front") -
// REP103's positive=CCW=left convention, so headingRad/steerRad below need
// no sign flip to match.
function _arrowTip(cx,cy,angleRad,length){
  return [cx-Math.sin(angleRad)*length, cy-Math.cos(angleRad)*length];
}
function _drawArrow(x,cx,cy,angleRad,length,color,lineWidth){
  const [tx,ty]=_arrowTip(cx,cy,angleRad,length);
  x.strokeStyle=color; x.fillStyle=color; x.lineWidth=lineWidth;
  x.beginPath(); x.moveTo(cx,cy); x.lineTo(tx,ty); x.stroke();
  const headAngle=0.4, headLen=Math.min(10,length*0.35);
  const back1=_arrowTip(tx,ty,angleRad+Math.PI-headAngle,headLen);
  const back2=_arrowTip(tx,ty,angleRad+Math.PI+headAngle,headLen);
  x.beginPath(); x.moveTo(tx,ty); x.lineTo(back1[0],back1[1]);
  x.lineTo(back2[0],back2[1]); x.closePath(); x.fill();
}

function drawLidar(scan,missionHeading,cmdVel){
  const c=document.getElementById("lidar"),x=c.getContext("2d");
  const W=c.width,H=c.height,cx=W/2,cy=H/2;
  x.clearRect(0,0,W,H); x.fillStyle="#0b0e13"; x.fillRect(0,0,W,H);
  const maxR=(scan&&scan.range_max)?Math.min(scan.range_max,8):8;
  const px=(Math.min(W,H)/2-16)/maxR;
  x.strokeStyle="#2a323e"; x.lineWidth=1;
  for(let r=1;r<=Math.floor(maxR);r++){x.beginPath();x.arc(cx,cy,r*px,0,6.2832);x.stroke();}
  x.beginPath();x.moveTo(cx,10);x.lineTo(cx,H-10);x.moveTo(10,cy);x.lineTo(W-10,cy);x.stroke();
  x.fillStyle="#95a2b3";x.font="10px monospace";
  x.fillText("front",cx+5,16); x.fillText(maxR.toFixed(0)+"m",cx+4,cy-maxR*px+12);
  if(scan){
    x.fillStyle="#e5a748";
    for(let i=0;i<scan.ranges.length;i++){
      const r=scan.ranges[i]; if(!r||r<=0.02)continue;
      // +LIDAR_FRONT_OFFSET_RAD: the render math itself (sin/cos below) is
      // self-consistent - angle 0 always lands at the top of the canvas. The
      // user reported the physical front rendering at the back (2026-09-09,
      // second report, distinct from the earlier left-right mirror fix), which
      // means the lidar's own angle_min=0 reference does not line up with the
      // vehicle's actual forward direction - a mounting/frame issue upstream
      // of this rendering code, not a rendering bug. Best guess is a clean
      // 180 degrees; if the real offset turns out to be some other angle,
      // change this constant (and mission_manager.py's matching one - see
      // that file's min_range_in_sector) rather than re-guessing the
      // sin/cos signs here again.
      const a=scan.angle_min+i*scan.angle_increment+Math.PI;
      const px_=cx-Math.sin(a)*r*px, py_=cy-Math.cos(a)*r*px;
      x.fillRect(px_-1.3,py_-1.3,2.6,2.6);
    }
  }

  // Heading overlay (2026-09-11): this plot is robot-body-frame ("front" is
  // always straight up, by construction of the scan render above), so this
  // is NOT "which way is the robot facing in the world" - there is no world
  // frame drawn here at all. It shows how far the vehicle has yawed away
  // from the reference heading it had when autonomous last engaged
  // (yaw_rel from heading_reference.py's /mission/heading) - a compass-
  // deviation needle, not a compass. Only meaningful once that reference
  // has actually been zeroed once (mission_logic.py's own drift-correction
  // depends on the same zeroing).
  if(missionHeading&&missionHeading.zeroed){
    _drawArrow(x,cx,cy,missionHeading.yaw_rel,Math.min(W,H)*0.28,"#6fd3ff",2);
  }
  // Steering/command overlay: mission_manager.py's live /cmd_vel - only
  // ever actually driven while in AUTONOMOUS (manual driving mixes sticks
  // directly on the ESP32, never touches this topic - see
  // esp32_firmware.ino's MANUAL branch), so this shows what the reactive
  // corridor-follower is CURRENTLY deciding to do, not a lookahead path -
  // there is no planned trajectory anywhere in this system yet (see
  // CLAUDE.md's Nav2 entry).
  if(cmdVel&&(Math.abs(cmdVel.linear)>0.01||Math.abs(cmdVel.angular)>0.01)){
    const reversing=cmdVel.linear<0;
    const len=28+Math.min(1,Math.abs(cmdVel.linear))*70;
    const angle=(reversing?Math.PI:0)+cmdVel.angular*0.9;
    _drawArrow(x,cx,cy,angle,len,reversing?"#e2695a":"#5fb587",3);
  }

  x.fillStyle="#5fb587"; x.fillRect(cx-3,cy-3,6,6);
}

function drawAttitude(imu){
  const c=document.getElementById("att"),x=c.getContext("2d");
  const W=c.width,H=c.height;
  x.clearRect(0,0,W,H); x.fillStyle="#0b0e13"; x.fillRect(0,0,W,H);
  if(!imu)return;
  const cx=W/2, cy=H/2, R=Math.min(W,H)/2-14;
  const roll=imu.roll*Math.PI/180, pitch=imu.pitch;
  x.save(); x.beginPath(); x.arc(cx,cy,R,0,6.2832); x.clip();
  x.translate(cx,cy); x.rotate(-roll); x.translate(0,pitch*2.2);
  x.fillStyle="#24384d"; x.fillRect(-R*2,-R*3,R*4,R*3);      // sky
  x.fillStyle="#3d2f1c"; x.fillRect(-R*2,0,R*4,R*3);          // ground
  x.strokeStyle="#e4e8ee"; x.lineWidth=2;
  x.beginPath(); x.moveTo(-R*2,0); x.lineTo(R*2,0); x.stroke();
  x.strokeStyle="#95a2b3"; x.lineWidth=1;
  for(let p=-30;p<=30;p+=10){ if(!p)continue; const y=-p*2.2;
    x.beginPath(); x.moveTo(-28,y); x.lineTo(28,y); x.stroke(); }
  x.restore();
  x.strokeStyle="#2a323e"; x.beginPath(); x.arc(cx,cy,R,0,6.2832); x.stroke();
  x.strokeStyle="#e5a748"; x.lineWidth=2;
  x.beginPath(); x.moveTo(cx-26,cy); x.lineTo(cx-8,cy);
  x.moveTo(cx+8,cy); x.lineTo(cx+26,cy); x.stroke();
}

async function tick(){
  let s;
  try{ s=await (await fetch("/api/state",{cache:"no-store"})).json(); }
  catch(e){ document.getElementById("dot").className="dot";
            document.getElementById("conn").textContent="disconnected"; return; }
  document.getElementById("dot").className="dot live";
  document.getElementById("conn").textContent=Object.keys(s).length+" topics";

  ["front_image|a_front","rear_image|a_rear","aim_image|a_aim",
   "debug_image|a_debug","scan|a_scan","imu|a_imu",
   "diagnostics|a_diag","sbus|a_sbus","door|a_door"]
   .forEach(p=>{const[k,id]=p.split("|");age(id,s[k]);});
  age("a_stage", s.current_stage, true);   // latched
  age("a_tune", s.tuning);
  const tn=s.tuning?s.tuning.value:null;
  document.getElementById("tuneEcho").textContent = tn
    ? `live on vehicle: scale ${tn.speed_scale.toFixed(2)} · max lin ${tn.max_linear.toFixed(2)} · max ang ${tn.max_angular.toFixed(2)}`
    : "serial bridge not reporting tuning state";

  renderSystem(s);
  renderRecording(s);
  drawLidar(s.scan&&!s.scan.stale?s.scan.value:null,
            s.mission_heading&&!s.mission_heading.stale?s.mission_heading.value:null,
            s.cmd_vel&&!s.cmd_vel.stale?s.cmd_vel.value:null);
  const imu=s.imu&&!s.imu.stale?s.imu.value:null;
  drawAttitude(imu);
  rows("imuTbl", imu?[
    ["roll",imu.roll.toFixed(2)+"°"],["pitch",imu.pitch.toFixed(2)+"°"],
    ["yaw",imu.yaw.toFixed(2)+"°"],
    ["gyro x/y/z",imu.gyro.join(" / ")],["accel x/y/z",imu.accel.join(" / ")],
    ["quat",imu.quat.join(", ")]]:[["status","no IMU data"]]);

  const st=s.current_stage?s.current_stage.value:null;
  document.getElementById("stageNum").textContent=(st===null)?"—":st;
  document.getElementById("stageName").textContent=
    st?(STAGES[st]||"unknown stage"):"not started";
  const tr=s.stage_transition?s.stage_transition.value:null;
  const so=s.sign_observation?s.sign_observation.value:null;
  const sc=s.sign_confirmed?s.sign_confirmed.value:null;
  rows("missionTbl",[
    ["last transition",tr?`${tr.previous}→${tr.current} ${tr.behavior}`:"—"],
    ["transition at",tr?tr.at:"—"],
    ["sign seen",so?`id ${so.sign_id} (${so.confidence})`:"—"],
    ["sign confirmed",sc?`id ${sc.sign_id} @ ${sc.at}`:"—"]]);

  const d=s.diagnostics?s.diagnostics.value:null;
  document.getElementById("espChips").innerHTML= d?
    `<span class="chip ${d.armed?"on":"off"}">${d.armed?"ARMED":"DISARMED"}</span>
     <span class="chip ${d.sbus_ok?"on":"off"}">SBUS ${d.sbus_ok?"OK":"DOWN"}</span>`
    :`<span class="chip off">NO LINK</span>`;

  // Autonomous status panel: read-only - autonomous is selected ONLY by
  // the physical SwB switch (2026-09-11, dashboard control removed by
  // explicit request), this just shows what the ESP32 reports it's
  // actually doing right now.
  const autoChips=document.getElementById("autoChips");
  if(d){
    autoChips.innerHTML=
      `<span class="chip ${d.autonomous_mode?"on":"off"}">${d.autonomous_mode?"AUTONOMOUS":"manual"}</span>
       <span class="chip ${d.estop_ok===false?"off":"on"}">${d.estop_ok===false?"E-STOP TRIPPED":"e-stop ok"}</span>`;
  }else{
    autoChips.innerHTML=`<span class="chip off">NO LINK</span>`;
  }
  const m=s.motor?s.motor.value:null;
  rows("espTbl",[["loop",d?d.loop_hz+" Hz":"—"],
    ["left DAC",m?`${m.left_dac}${m.left_reverse?" (rev)":""}`:"—"],
    ["right DAC",m?`${m.right_dac}${m.right_reverse?" (rev)":""}`:"—"]]);

  const sb=s.sbus?s.sbus.value:null;
  document.getElementById("sbus").innerHTML = (sb&&sb.length)?
    sb.map((v,i)=>{const p=Math.max(0,Math.min(100,(v-172)/(1811-172)*100));
      return `<div class="sbus-row"><span>CH${i+1}</span>
        <span class="bar"><i style="left:0;width:${p}%"></i></span><b>${v}</b></div>`;}).join("")
    : `<span class="sub">no SBUS data (receiver unbound)</span>`;

  const dr=s.door?s.door.value:null, ob=s.obstacles?s.obstacles.value:null;
  rows("doorTbl",[
    ["door direction",dr?({"1":"RIGHT","-1":"LEFT","0":"unknown"}[String(dr.direction)]||"unknown"):"—"],
    ["blocked",dr?(dr.blocked_fraction*100).toFixed(0)+"%":"—"],
    ["period",dr?dr.period_s.toFixed(2)+" s":"—"],
    ["clear for",dr?dr.clear_elapsed_s.toFixed(2)+" s":"—"],
    ["ground obstacles",ob?(ob.length?ob.map(o=>`${o.range_m}m@${o.bearing_deg}°`).join(", "):"none"):"—"]]);
}
// Reset buttons: force that camera's worker to release+reopen its device
// immediately, for when it looks stuck rather than cleanly disconnected.
// Does NOT drop/reopen the <img> stream itself - the worker publishing
// again on the same topic is enough, no page-side reconnect needed.
document.querySelectorAll("button.pick[data-cam]").forEach(b=>{
  b.addEventListener("click",async()=>{
    b.disabled=true;
    try{ await fetch("/camera/reset?cam="+b.dataset.cam,{cache:"no-store"}); }catch(e){}
    setTimeout(()=>{b.disabled=false;},1500);
  });
});

// Solo view: driving over a slow/high-latency link (Tailscale from far
// away, say) means three simultaneous MJPEG streams compete for the same
// limited bandwidth as the one that actually matters for driving. Picking
// "front only" (or any one camera) blanks the other two <img> src so the
// browser drops those HTTP connections entirely - the server stops having
// anyone to push those bytes to, freeing that bandwidth for the one stream
// still open. Remembered in localStorage so a reload doesn't silently go
// back to all three and reintroduce the lag.
const CAM_STREAM_SRC={front:"/stream/front.mjpg",rear:"/stream/rear.mjpg",aim:"/stream/aim.mjpg"};
function applySoloView(which){
  for(const name of Object.keys(CAM_STREAM_SRC)){
    const img=document.getElementById("camImg_"+name);
    const cell=document.getElementById("camCell_"+name);
    const show=(which==="all"||which===name);
    if(show){
      if(!img.src||img.src.indexOf(CAM_STREAM_SRC[name])===-1) img.src=CAM_STREAM_SRC[name];
      cell.style.opacity="1";
    }else{
      img.src="";  // drops the open connection - see comment above
      cell.style.opacity=".35";
    }
  }
  document.querySelectorAll("button.solo-view").forEach(b=>{
    b.classList.toggle("active", b.dataset.solo===which);
  });
  try{ localStorage.setItem("teknofest_solo_view", which); }catch(e){}
}
document.querySelectorAll("button.solo-view").forEach(b=>{
  b.addEventListener("click",()=>applySoloView(b.dataset.solo));
});
let initialSolo="all";
try{ initialSolo=localStorage.getItem("teknofest_solo_view")||"all"; }catch(e){}
if(initialSolo!=="all") applySoloView(initialSolo);

// Analysis views share ONE stream connection - browsers allow only ~6 per
// host, and four always-open MJPEG streams would starve the state polling.
const VIEW_NOTE={
  debug:"sign detector - red-ring localisation + ONNX digit classification",
  segmentation:"drivable ground (green) + obstacles standing on it, with range/bearing",
  aimdebug:"laser boresight vs detected target; requires the AIM camera to be selected"
};
const analysisImg=document.getElementById("analysisView");
const analysisNote=document.getElementById("analysisNote");
analysisNote.textContent=VIEW_NOTE.debug;
document.querySelectorAll("button.view").forEach(b=>{
  b.addEventListener("click",()=>{
    document.querySelectorAll("button.view").forEach(o=>o.classList.remove("active"));
    b.classList.add("active");
    const v=b.dataset.view;
    // Drop the old stream before opening the new one, or the abandoned
    // connection lingers and counts against the per-host limit.
    analysisImg.src="";
    analysisImg.src="/stream/"+v+".mjpg";
    analysisNote.textContent=VIEW_NOTE[v]||"";
  });
});

// One row per thing that can be down, so the page doubles as the "what is
// broken" answer rather than only showing data when everything works.
function sysRow(name,state,detail){
  return `<div class="sysitem"><span class="led ${state}"></span>
    <span class="nm">${name}</span><span class="vl">${detail}</span></div>`;
}
function renderSystem(s){
  const live=k=>s[k]&&!s[k].stale;
  const cs=s.camera_status?s.camera_status.value:null;
  const d=s.diagnostics?s.diagnostics.value:null;
  const out=[];

  if(cs){
    for(const [n,c] of Object.entries(cs)){
      out.push(sysRow("cam "+n, c.present?(c.streaming?"ok":"warn"):"bad",
        c.present?(c.streaming?"streaming ("+c.frames+"f)":(c.error||"not streaming")):"missing"));
    }
  }else{
    ["front","aim","rear"].forEach(n=>out.push(sysRow("cam "+n,"bad","no manager")));
  }
  out.push(sysRow("lidar /scan", live("scan")?"ok":"bad",
    live("scan")?s.scan.value.ranges.length+" beams":"down"));
  out.push(sysRow("ESP32 link", live("diagnostics")?"ok":"bad",
    d?d.loop_hz+" Hz":"down"));
  out.push(sysRow("IMU", live("imu")?"ok":"bad", live("imu")?"publishing":"down"));
  out.push(sysRow("SBUS receiver", d?(d.sbus_ok?"ok":"bad"):"bad",
    d?(d.sbus_ok?"bound":"unbound"):"—"));
  out.push(sysRow("drive armed", d?(d.armed?"warn":"ok"):"bad",
    d?(d.armed?"ARMED":"disarmed"):"—"));
  out.push(sysRow("sign detector", live("debug_image")?"ok":"bad",
    live("debug_image")?"running":"no debug feed"));
  out.push(sysRow("stage manager", s.current_stage?"ok":"bad",
    s.current_stage?("stage "+s.current_stage.value):"—"));
  out.push(sysRow("door tracker", live("door")?"ok":"warn",
    live("door")?"running":"not started"));
  out.push(sysRow("path segmentation", live("obstacles")?"ok":"warn",
    live("obstacles")?"running":"not started"));
  out.push(sysRow("aim tracker", live("aimdebug")?"ok":"warn",
    live("aimdebug")?"tracking":((cs&&cs.aim&&cs.aim.streaming)?"no lock yet":"needs aim cam")));
  document.getElementById("sysstate").innerHTML=out.join("");

  if(cs){
    const parts=["front","rear","aim"].map(n=>{
      const c=cs[n];
      if(!c) return n+": ?";
      if(!c.present) return n+": missing";
      return n+": "+(c.streaming?"live":"stalled");
    });
    document.getElementById("camNote").textContent = parts.join("  ·  ");
    for(const n of ["front","rear","aim"]){
      const c=cs[n];
      const el=document.getElementById("camState_"+n);
      if(el && c){
        el.textContent = c.present?(c.streaming?"live ("+c.frames+"f)":(c.error||"stalled")):"missing";
        el.className = "camstate "+(c.present?(c.streaming?"ok":"warn"):"bad");
      }
    }
  }
}
// --- drive tuning ---
const T={scale:["t_scale","v_scale"],lin:["t_lin","v_lin"],ang:["t_ang","v_ang"]};
function tuneLabels(){
  for(const [sl,lb] of Object.values(T)){
    document.getElementById(lb).textContent=
      (document.getElementById(sl).value/100).toFixed(2);
  }
}
Object.values(T).forEach(([sl])=>
  document.getElementById(sl).addEventListener("input",tuneLabels));
tuneLabels();

async function applyTuning(scale,lin,ang){
  const q=`/tune?speed_scale=${scale}&max_linear=${lin}&max_angular=${ang}`;
  try{ await fetch(q,{cache:"no-store"}); }catch(e){}
}
document.getElementById("t_apply").addEventListener("click",()=>{
  applyTuning(document.getElementById("t_scale").value/100,
              document.getElementById("t_lin").value/100,
              document.getElementById("t_ang").value/100);
});
document.getElementById("t_stop").addEventListener("click",()=>{
  // Not an emergency stop - the real e-stop must stay independent of this
  // computer. This only zeroes the scale so /cmd_vel stops reaching the motors.
  document.getElementById("t_scale").value=0; tuneLabels();
  applyTuning(0,-1,-1);
});

// --- recording ---
function renderRecording(s){
  const r=s.recording?s.recording.value:null;
  const btn=document.getElementById("recBtn"), status=document.getElementById("recStatus");
  const active=r&&r.active;
  btn.textContent=active?"stop":"record";
  btn.style.background=active?"#6b3030":"#3a1e1e";
  status.textContent=active?`recording to ${r.dir}`:"not recording";
  const cams=(r&&r.cameras)?r.cameras:{};
  const names=Object.keys(cams);
  rows("recTbl", names.length
    ? names.map(n=>[n, cams[n]])
    : [["status", active?"starting…":"—"]]);
}
let recBusy=false;
document.getElementById("recBtn").addEventListener("click", async ()=>{
  if(recBusy) return;
  recBusy=true;
  const btn=document.getElementById("recBtn");
  const starting = btn.textContent==="record";
  try{
    await fetch(starting?"/record/start":"/record/stop", {cache:"no-store"});
  }catch(e){}
  recBusy=false;
  tick();
});

setInterval(tick,200); tick();
</script></body></html>
"""


def main():
    rclpy.init()
    node = TelemetryNode()
    NODE_REF["node"] = node
    port = int(node.get_parameter("port").value)

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()  # avoid an orphaned recorder/bag subprocess if the dashboard itself is stopped mid-recording
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
