#!/usr/bin/env python3
"""
Owns all three cameras and publishes ALL of them simultaneously.

Rewritten 2026-09-10 - the single-active-camera design below is now
obsolete. It existed because all three cameras used to share one USB 2.0
hub (measured: any pair left the second at 0 fps, not fixable in
software). That is no longer true: moving the front camera to the Pi's
USB-C OTG port put it on a THIRD, fully independent USB host controller
(confirmed via udevadm ID_PATH: front=xhci-hcd.15.auto, rear=fc800000.usb,
aim=xhci-hcd.3.auto - three different controllers, not three ports on one
hub). Each camera now has its own dedicated ~480Mbps budget, so there is
no longer a hardware reason only one can stream at a time - and driving
off the front camera while also watching/recording rear+aim needs exactly
that.

Each camera runs its own independent capture thread (CameraWorker) with
its own retry-on-failure loop, so one camera dropping out can never affect
the other two - matches the same per-camera-thread pattern already
validated in mission_recorder.py.

  /camera/reset    (std_msgs/String)  "front" | "rear" | "aim" | "all" -
                    forces that camera's worker to release and reopen its
                    device immediately, without waiting for the normal
                    REOPEN_BACKOFF_S retry timer. For when a camera looks
                    stuck rather than cleanly disconnected.
  /camera/status   (std_msgs/String)  JSON: per-camera health (present,
                    streaming, frame count, last error)
  /front/image_raw /aim/image_raw /rear/image_raw   (sensor_msgs/Image) -
                    all three publish continuously and simultaneously now.

Note on /camera/reset and a genuinely hung read: CameraWorker.run() only
checks for a pending reset between frames - if cv2.VideoCapture.read()
itself is blocked (a real device hang, not just "disconnected", which
OpenCV/V4L2 normally reports quickly as a failed read), the reset will not
take effect until that call returns. Accepted as a reasonable limitation
for a straightforward per-thread capture loop; a subprocess-per-camera
model would allow a true forced kill if this ever proves insufficient in
practice.
"""
import json
import os
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

CAMERAS = {
    "front": "/dev/cam_front",
    "aim": "/dev/cam_aim",
    "rear": "/dev/cam_rear",
}

# Per-camera capture profile: (width, height, fps).
#
# The aim camera runs at full resolution on purpose. The laser target is
# 36 cm across engaged at 10 m, which is only ~2.06 deg - about 21 px in a
# 640-wide frame, giving an inner ring of ~3.5 px radius. That is below what
# Hough circle detection can resolve, so target tracking simply cannot work
# at 640x480. At 2560 wide the target is ~85 px across (inner ring ~14 px),
# which is workable.
#
# fps raised 10->25 (2026-09-10): a slower feed means the closed loop
# reacts to a position the target already left, and corrects less often -
# exactly backwards for something that needs to converge and hold a lock.
# 25 is a request, not a guarantee - the camera's real sensor readout time
# at full 2560x1440 may not sustain it; _open() below logs the actual
# reported fps so this is visible rather than silently assumed.
PROFILES = {
    "front": (640, 480, 30),
    "rear": (640, 480, 30),
    "aim": (2560, 1440, 25),
}
DEFAULT_PROFILE = (640, 480, 30)
REOPEN_BACKOFF_S = 2.0


class CameraWorker:
    """One camera's independent capture+publish loop - own thread, own
    retry state, own ROS publisher. A failure here (device gone, read
    error) can never block or affect the other two cameras."""

    def __init__(self, node: "CameraManager", name: str, device: str):
        self.node = node
        self.name = name
        self.device = device
        self.profile = PROFILES.get(name, DEFAULT_PROFILE)
        self.publisher = node.create_publisher(Image, f"/{name}/image_raw", 5)
        self.cap = None
        self.frames = 0
        self.last_frame_t = 0.0
        self.last_error = ""
        self._reset_requested = threading.Event()

    def request_reset(self):
        self._reset_requested.set()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.last_error = f"cannot open {self.device}"
            return None
        width, height, fps = self.profile
        # MJPG matters even for a single stream: it is the difference
        # between ~30 fps and a stuttering feed on this bus, and at
        # 2560x1440 it is the difference between working and not.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if actual != (width, height):
            self.node.get_logger().warn(
                f"{self.name}: asked for {width}x{height}, got {actual[0]}x{actual[1]}"
            )
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        if actual_fps and abs(actual_fps - fps) > 1.0:
            self.node.get_logger().warn(
                f"{self.name}: asked for {fps}fps, camera reports {actual_fps:.1f}fps "
                f"(driver-reported, not measured - see /camera/status frame count)"
            )
        self.last_error = ""
        return cap

    def run(self):
        while rclpy.ok():
            if self._reset_requested.is_set():
                self._reset_requested.clear()
                if self.cap is not None:
                    self.cap.release()
                    self.cap = None
                self.node.get_logger().info(f"{self.name}: reset requested, reopening")

            if self.cap is None:
                self.cap = self._open()
                if self.cap is None:
                    time.sleep(REOPEN_BACKOFF_S)
                    continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.last_error = f"read failed on {self.name}"
                self.cap.release()
                self.cap = None
                time.sleep(REOPEN_BACKOFF_S)
                continue

            msg = self.node.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = f"camera_{self.name}"
            self.publisher.publish(msg)
            self.frames += 1
            self.last_frame_t = time.time()

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def status(self):
        now = time.time()
        return {
            "device": self.device,
            "present": os.path.exists(self.device),
            "streaming": (now - self.last_frame_t) < 2.0 if self.last_frame_t else False,
            "frames": self.frames,
            "error": self.last_error,
        }


class CameraManager(Node):
    def __init__(self):
        super().__init__("camera_manager")
        self.bridge = CvBridge()

        self.workers = {name: CameraWorker(self, name, dev) for name, dev in CAMERAS.items()}
        self.status_pub = self.create_publisher(String, "/camera/status", 5)
        self.create_subscription(String, "/camera/reset", self.on_reset, 5)

        for worker in self.workers.values():
            threading.Thread(target=worker.run, daemon=True).start()

        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"camera_manager started - {len(self.workers)} cameras streaming simultaneously "
            f"(one thread each, independent USB controllers)"
        )

    def on_reset(self, msg: String):
        name = msg.data.strip().lower()
        if name == "all":
            for worker in self.workers.values():
                worker.request_reset()
            self.get_logger().info("reset requested for all cameras")
            return
        worker = self.workers.get(name)
        if worker is None:
            self.get_logger().warn(f"ignoring reset for unknown camera '{name}'")
            return
        worker.request_reset()
        self.get_logger().info(f"reset requested for {name}")

    def publish_status(self):
        status = {name: worker.status() for name, worker in self.workers.items()}
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for worker in node.workers.values():
            worker.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
