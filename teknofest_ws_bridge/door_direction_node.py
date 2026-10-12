#!/usr/bin/env python3
"""
Determines which way the sliding door is moving, from the front camera.

Stage 6 is worth 50 points and is lost by contact, not by slowness. The
committed strategy is to hug the right wall and enter only after the door
has cleared to the RIGHT: a left exit or an ambiguous read leaves far less
usable window and risks a strike. That needs the one thing a 10 Hz lidar
gives only coarsely - the door's direction of travel and the phase of its
cycle - so the front camera measures it directly.

All logic lives in door_tracking.DoorTracker so it can be tested without a
robot, a camera, or a real door; this node is ROS plumbing only. See that
module for the method and its physical reasoning.

Verified on synthesised sweeps (not on a real door): direction correct on
32/33 right-moving and 31/31 left-moving frames with zero misreads, no
direction reported on a static scene, and a measured cycle period of 1.83 s
against a 1.83 s ground truth.

Publishes /door/state (std_msgs/Float32MultiArray):
    [0] direction        -1.0 = moving left, +1.0 = moving right, 0.0 = unknown
    [1] centroid_x_norm  horizontal centre of motion, 0.0-1.0 across the ROI
    [2] velocity_px_s    signed horizontal speed of the motion centroid
    [3] blocked_fraction 0.0-1.0 of the gate currently occluded
    [4] period_s         measured full cycle, 0.0 until a cycle completes
    [5] clear_elapsed_s  seconds the gate has been continuously clear
    [6] blocked          1.0/0.0 - the tracker's own hysteresis-debounced
                          blocked/clear state (added 2026-09-11 for
                          mission_manager.py's SlidingDoorController;
                          additive, indices 0-5 unchanged)

NOT YET VALIDATED against the real door - none has been filmed, and the
spec's stated "20 m/s" is not physically credible at parkour scale (flagged
for official Q&A). Nothing here hard-codes a speed: the period is measured
on site, which is what the 5-minute self-trial window is for. Per
CLAUDE.md's working agreement this must not gate vehicle motion until it
has been confirmed against the physical mechanism, and lidar stays
authoritative for whether the gate is actually passable.
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from door_tracking import DoorTracker

# Gate ROI as fractions of the frame - the band of corridor the door sweeps.
# The door hangs above ground level, so the lowest part of the frame is mostly
# floor and contributes noise rather than signal. Tune on site.
ROI_X_MIN, ROI_X_MAX = 0.05, 0.95
ROI_Y_MIN, ROI_Y_MAX = 0.15, 0.80


class DoorDirectionNode(Node):
    def __init__(self):
        super().__init__('door_direction_node')
        self.bridge = CvBridge()
        self.tracker = DoorTracker()
        self.pub = self.create_publisher(Float32MultiArray, '/door/state', 10)
        self.debug_pub = self.create_publisher(Image, '/door/debug_image', 1)
        self.sub = self.create_subscription(Image, '/front/image_raw', self.on_image, 10)
        self.get_logger().info('door_direction_node started, waiting for /front/image_raw')

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'image conversion failed: {e}')
            return

        h, w = frame.shape[:2]
        x0, x1 = int(w * ROI_X_MIN), int(w * ROI_X_MAX)
        y0, y1 = int(h * ROI_Y_MIN), int(h * ROI_Y_MAX)
        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return

        state = self.tracker.update(roi, time.monotonic())

        out = Float32MultiArray()
        out.data = [float(value) for value in state.as_list()]
        self.pub.publish(out)

        if self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(frame, (x0, y0, x1, y1), state, msg)

    def _publish_debug(self, frame, roi_box, state, source_msg):
        x0, y0, x1, y1 = roi_box
        vis = frame.copy()
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 200, 255), 2)
        if self.tracker.last_motion_mask is not None:
            mask = cv2.cvtColor(self.tracker.last_motion_mask, cv2.COLOR_GRAY2BGR)
            if mask.shape == vis[y0:y1, x0:x1].shape:
                vis[y0:y1, x0:x1] = cv2.addWeighted(vis[y0:y1, x0:x1], 0.7, mask, 0.3, 0)

        heading = {1.0: 'moving RIGHT', -1.0: 'moving LEFT'}.get(
            state.direction, 'direction unknown'
        )
        occupancy = 'BLOCKED' if state.blocked else 'CLEAR'
        cv2.putText(
            vis,
            f'{heading} | {occupancy} {state.blocked_fraction:.2f} | period {state.period_s:.2f}s',
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2,
        )
        out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out.header = source_msg.header
        self.debug_pub.publish(out)


def main():
    rclpy.init()
    node = DoorDirectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
