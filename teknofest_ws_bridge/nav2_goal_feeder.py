#!/usr/bin/env python3
"""
Keeps Nav2 actually driving by continuously feeding it a "drive forward"
goal in the robot's own moving frame (2026-09-11).

This project's Nav2 setup deliberately has no persistent map (course
unknown until race day - see robot_description/config/nav2_params_hardware.yaml's
header comment), so there is no "click a point on a map" goal source.
Without something like this, launching nav2_hardware.launch.py brings up
every server correctly but the robot never moves at all -
bt_navigator/NavigateToPose just waits forever for a goal nobody sends.

Design: repeatedly send a NavigateToPose goal GOAL_DISTANCE_M straight
ahead of the robot's CURRENT position and heading (looked up via tf2 in
the odom frame at send time), not one single fixed point - as the robot
advances, each new goal is fresh relative to wherever it now is. A new
goal is sent once the previous one finishes (succeeded OR aborted - e.g.
Nav2 gave up because the local costmap sees no way through) or after
GOAL_TIMEOUT_S with no result, whichever comes first, so the robot can
never just get stuck waiting on one stale goal forever.

This makes Nav2 behave like an obstacle-avoiding "always advance" system -
a genuine capability upgrade over mission_manager.py's simpler
wall-following heuristic, since Nav2's local costmap + MPPI controller can
plan an actual path around an obstacle rather than just slow/stop for one
dead ahead. It is explicitly NOT objective-driven navigation to a real
course goal ("go to the finish") - there is still no map, so this only
ever knows "keep going forward, avoid what's in front of you." Real
goal-directed navigation needs a real map + localization (AMCL or
equivalent), a separate, bigger step from this.

UNTESTED - like the rest of the Nav2 hardware stack, this has never run
against a real robot. Run alongside nav2_hardware.launch.py, never
alongside mission_manager.py (both would end up driving through Nav2's
own /cmd_vel output either way, but mission_manager.py's own /cmd_vel
publish would still fight Nav2's for the same topic - see that launch
file's own docstring on this).
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

GOAL_DISTANCE_M = 3.0      # how far ahead (along current heading) each goal is placed
GOAL_TIMEOUT_S = 20.0      # give up waiting on one goal and send a fresh one after this long
CHECK_INTERVAL_S = 1.0     # how often to check whether a new goal is needed


class Nav2GoalFeeder(Node):
    def __init__(self):
        super().__init__('nav2_goal_feeder')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._active = False
        self._goal_sent_time = 0.0
        self.create_timer(CHECK_INTERVAL_S, self._check_and_send)
        self.get_logger().info(
            f'nav2_goal_feeder started - sending {GOAL_DISTANCE_M}m-ahead goals '
            f'to keep Nav2 driving with no map'
        )

    def _check_and_send(self):
        if self._active:
            elapsed = time.monotonic() - self._goal_sent_time
            if elapsed < GOAL_TIMEOUT_S:
                return  # still waiting on the current goal
            self.get_logger().warning(f'goal timed out after {elapsed:.1f}s - sending a fresh one')
            self._active = False
        self._send_forward_goal()

    def _send_forward_goal(self):
        if not self.client.server_is_ready():
            self.get_logger().warning(
                'navigate_to_pose action server not up yet', throttle_duration_sec=5.0
            )
            return
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(
                f'no odom->base_footprint transform yet: {e}', throttle_duration_sec=5.0
            )
            return

        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = t.transform.translation.x + GOAL_DISTANCE_M * math.cos(yaw)
        goal_pose.pose.position.y = t.transform.translation.y + GOAL_DISTANCE_M * math.sin(yaw)
        goal_pose.pose.orientation = t.transform.rotation  # keep facing the same way

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self._active = True
        self._goal_sent_time = time.monotonic()
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f'sent goal {GOAL_DISTANCE_M}m ahead at '
            f'({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning('goal rejected by Nav2')
            self._active = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, _future):
        # Whatever the outcome (succeeded/aborted/canceled), the next timer
        # tick sends a fresh goal from wherever the robot now is.
        self._active = False


def main():
    rclpy.init()
    node = Nav2GoalFeeder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
