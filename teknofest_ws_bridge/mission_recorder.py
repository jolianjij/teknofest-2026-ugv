#!/usr/bin/env python3
"""
Standalone recording process for the dashboard's "Record" button - launched
and killed as a subprocess by web_dashboard.py (not a long-running ROS
node normally kept up), one process per recording session.

Rewritten 2026-09-10 to stop opening the camera devices a second time.
camera_manager_node.py now holds all three cameras open continuously and
already publishes every frame on /front/image_raw, /rear/image_raw,
/aim/image_raw - so this process just subscribes to those topics and writes
whatever arrives to disk. This removes BOTH real unknowns the old direct-
capture design carried (whether the USB bandwidth budget supports a second
simultaneous open per camera, and whether the V4L2/UVC driver even permits
a second process to open a device camera_manager_node.py already holds) -
there is no longer a second open at all, so neither question matters. It
also means recording can never contend with camera_manager_node.py for USB
bandwidth or double the capture-side CPU/decode cost - the only extra cost
versus the live preview alone is the mp4 encode itself.

One real behavior change from this: if camera_manager_node.py is not
running, there is nothing to subscribe to and every camera's status stays
"waiting for camera_manager" - correctly reflecting that recording is
entirely downstream of it now, rather than a separate path that used to
work independently. Also: front/rear now record at whatever resolution
camera_manager_node.py actually publishes (currently 640x480, its live-
preview profile) instead of requesting a separate 1280x720 capture profile
directly from the device - that old request may not even have been honored
by these cameras' native modes; this way the recording matches what was
actually on the wire, not a hopeful separate ask. The aim camera still
gets downscaled on write (see AIM_RECORD_MAX_WIDTH below) since its live
profile is a deliberately large 2560x1440 for target-detection precision,
which is excessive to encode to disk for a multi-minute run.

Each camera's frames are written by its own subscription callback with its
own VideoWriter - one camera falling behind or momentarily losing frames
can never block or corrupt another's file. A gap in frames (camera_manager
mid-reset, say) just means no new frames get written until they resume -
nothing to corrupt, since there is no capture handle here to reopen. The
per-camera "recording_status.json" file mission_recorder.py has always
written is unchanged in shape, so the dashboard's status table needs no
changes.

Usage: mission_recorder.py <output_dir>
Runs until SIGTERM/SIGINT, at which point every writer is released cleanly
so the .mp4 files are properly finalized rather than truncated/corrupt.
"""
import json
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_TOPICS = {
    "front": "/front/image_raw",
    "rear": "/rear/image_raw",
    "aim": "/aim/image_raw",
}
# camera_manager_node.py's PROFILES fps, for VideoWriter's declared playback
# rate - the actual arrival rate can run a little under this (see that
# file's own comment on the aim camera's real sensor readout time), which
# only affects apparent playback speed, not correctness.
RECORD_FPS = {"front": 30, "rear": 30, "aim": 25}
# The aim camera's live profile is 2560x1440 (needed for target-detection
# precision at a standstill, see camera_manager_node.py) - excessive to
# encode to disk continuously for a multi-minute mission. Downscaled on
# write only; front/rear are already below this and pass through untouched.
AIM_RECORD_MAX_WIDTH = 1280
STALE_AFTER_S = 3.0


class _CameraSink:
    """One camera's write side: lazily-opened VideoWriter + frame/health
    bookkeeping. Every method is called only from this camera's own ROS
    subscription callback except status reads, which take the lock."""

    def __init__(self, name: str, out_dir: Path):
        self.name = name
        self.out_dir = out_dir
        self.writer = None
        self.writer_size = None
        self.frame_count = 0
        self.last_frame_t = 0.0
        self.lock = threading.Lock()

    def write(self, frame):
        if self.name == "aim" and frame.shape[1] > AIM_RECORD_MAX_WIDTH:
            scale = AIM_RECORD_MAX_WIDTH / float(frame.shape[1])
            frame = cv2.resize(frame, None, fx=scale, fy=scale,
                                interpolation=cv2.INTER_AREA)
        h, w = frame.shape[:2]
        with self.lock:
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                path = self.out_dir / f"{self.name}.mp4"
                self.writer = cv2.VideoWriter(
                    str(path), fourcc, RECORD_FPS[self.name], (w, h)
                )
                self.writer_size = (w, h)
                print(f"{self.name}: recording to {path} ({w}x{h})", flush=True)
            elif (w, h) != self.writer_size:
                # camera_manager_node.py published a different resolution
                # than the first frame (e.g. a reset landing on a different
                # mode) - resize to match the file already opened so this
                # stays one continuous file per camera for the whole
                # mission instead of starting a new segment.
                frame = cv2.resize(frame, self.writer_size)
            self.writer.write(frame)
            self.frame_count += 1
            self.last_frame_t = time.time()

    def status_text(self, now: float) -> str:
        with self.lock:
            if self.last_frame_t == 0.0:
                return "waiting for camera_manager"
            if now - self.last_frame_t > STALE_AFTER_S:
                return f"stalled ({self.frame_count} frames so far)"
            return f"recording ({self.frame_count}f)"

    def final_status_text(self) -> str:
        with self.lock:
            return f"stopped ({self.frame_count} frames)"

    def close(self):
        with self.lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None


class MissionRecorderNode(Node):
    def __init__(self, out_dir: Path):
        super().__init__("mission_recorder")
        self.out_dir = out_dir
        self.bridge = CvBridge()
        self.sinks = {name: _CameraSink(name, out_dir) for name in CAMERA_TOPICS}

        for name, topic in CAMERA_TOPICS.items():
            self.create_subscription(
                Image, topic, self._make_callback(name), qos_profile_sensor_data
            )

        self.create_timer(1.0, self._write_status)

    def _make_callback(self, name: str):
        sink = self.sinks[name]

        def _cb(msg: Image):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                return
            sink.write(frame)

        return _cb

    def _write_status(self):
        now = time.time()
        cams = {name: sink.status_text(now) for name, sink in self.sinks.items()}
        status_path = self.out_dir / "recording_status.json"
        status_path.write_text(json.dumps({"cameras": cams, "recording": True}, indent=2))

    def finalize(self):
        cams = {}
        for name, sink in self.sinks.items():
            cams[name] = sink.final_status_text()
            sink.close()
        status_path = self.out_dir / "recording_status.json"
        status_path.write_text(json.dumps({"cameras": cams, "recording": False}, indent=2))


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: mission_recorder.py <output_dir>")
    out_dir = Path(sys.argv[1])
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = MissionRecorderNode(out_dir)
    print(
        f"recording to {out_dir} - subscribing to camera_manager_node.py's "
        f"image topics (no second device open)",
        flush=True,
    )

    stop_event = threading.Event()

    def _handle_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGTERM, _handle_stop)
    signal.signal(signal.SIGINT, _handle_stop)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    while not stop_event.is_set():
        stop_event.wait(0.5)

    node.finalize()
    rclpy.shutdown()
    spin_thread.join(timeout=5.0)
    print("recording stopped, all files finalized", flush=True)


if __name__ == "__main__":
    main()
