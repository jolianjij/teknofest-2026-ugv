#!/usr/bin/env python3
"""
Captures front-camera stills for segmentation training. Runs on the Pi.

The CNN in cnn_model.py has never been trained because terrain appearance
cannot be synthesised the way the sign design could - it needs real frames
of real ground. This is the collection step.

Saves one JPEG every SAVE_EVERY frames so a slow walk around produces a
varied set rather than hundreds of near-identical frames from one viewpoint.
Viewpoint and lighting variety matter far more than frame count: 200 frames
across different ground, angles and light beat 2000 from one spot.

The camera manager must have `front` selected (its default) - only one
camera can stream at a time on the current USB2 hub.

    python3 capture_frames.py --ros-args -p out_dir:=~/teknofest_ws/seg_frames
"""
import os
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SAVE_EVERY = 8


class FrameCapture(Node):
    def __init__(self):
        super().__init__("seg_frame_capture")
        self.declare_parameter("out_dir", "~/teknofest_ws/seg_frames")
        self.declare_parameter("save_every", SAVE_EVERY)

        self.out_dir = Path(os.path.expanduser(str(self.get_parameter("out_dir").value)))
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.save_every = int(self.get_parameter("save_every").value)

        self.bridge = CvBridge()
        self.seen = 0
        self.saved = len(list(self.out_dir.glob("*.jpg")))  # resume rather than overwrite
        self.create_subscription(
            Image, "/front/image_raw", self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"saving every {self.save_every} frames to {self.out_dir} "
            f"({self.saved} already present)"
        )

    def on_image(self, msg: Image):
        self.seen += 1
        if self.seen % self.save_every:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"conversion failed: {e}")
            return
        path = self.out_dir / f"frame_{self.saved:05d}.jpg"
        cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        self.saved += 1
        if self.saved % 25 == 0:
            self.get_logger().info(f"{self.saved} frames saved")


def main():
    rclpy.init()
    node = FrameCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"stopped with {node.saved} frames")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
