#!/usr/bin/env python3
"""
Zeroes a heading/attitude reference the instant the vehicle enters autonomous
mode, and latches (armed && autonomous_mode && estop_ok) into a single
"mission active" boolean that mission_manager.py starts/stops driving on.

Why a software reference instead of re-zeroing the BNO055 itself: the
sensor's own zero point is shared by every consumer, manual driving included.
Re-zeroing it on every autonomous engagement would reset it out from under
manual mode and anything else reading /esp32/imu. Instead this node captures
the IMU orientation at the moment autonomous engages and publishes everything
downstream RELATIVE to that captured reference - so at the instant of
engagement /mission/heading reads (0, 0, 0), matching "reset the gyro to
000 in everywhere" without touching the sensor or any other consumer.

Re-zeros on every rising edge, not just the first: disarming/switching back
to manual and re-engaging autonomous later (e.g. repositioning for another
attempt) captures a fresh reference each time, matching the described
workflow ("I will put the robot [at the start], then switch to autonomous")
as a repeatable per-attempt action, not a one-time boot-time calibration.

Fails safe on stale or missing /esp32/diagnostics: if the serial link to the
ESP32 drops (or the diagnostics rate falls well below its normal ~20Hz),
/mission/active is forced to False rather than latching whatever the last
known armed/autonomous state happened to be. mission_manager.py should still
apply its own independent staleness check on /mission/active - this is a
defense-in-depth chain (ESP32 cmd_vel timeout -> mission_manager's own
/mission/active staleness check -> this node's diagnostics staleness check),
not a single point that must never fail.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32MultiArray

# If /esp32/diagnostics hasn't arrived in this long, treat the link as dead
# and fail mission_active to False. The ESP32 publishes at ~20Hz
# (PUBLISH_PERIOD_MS=50 in config.h) in normal operation - this is a generous
# multiple of that, not a tight bound, since USB/serial jitter is expected.
DIAG_STALE_S = 1.0


def quat_to_euler(w, x, y, z):
    """Standard aerospace ZYX Euler extraction (roll, pitch, yaw), radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # guard asin() domain against float noise at the poles
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HeadingReferenceNode(Node):
    def __init__(self):
        super().__init__('heading_reference')

        self.reference_rpy = None       # (roll, pitch, yaw) captured at last engagement, or None if never engaged
        self.latest_rpy = (0.0, 0.0, 0.0)
        self._prev_mission_active = False

        self._armed = False
        self._autonomous_mode = False
        self._last_diag_time = None

        self.heading_pub = self.create_publisher(Float32MultiArray, '/mission/heading', 10)
        self.active_pub = self.create_publisher(Bool, '/mission/active', 10)
        # Tells stage_manager_node (ugv_stage_sign_detector package) to drop
        # back to its initial stage at the start of each fresh autonomous
        # attempt - without this a second attempt in the same process
        # lifetime (disarm, reposition, re-arm) stays stuck on whatever
        # stage the previous attempt reached and rejects sign 1 as
        # out-of-order. See stage_manager_node.py's _reset_callback.
        self.reset_stage_pub = self.create_publisher(Empty, '/mission/reset_stage_sequence', 10)

        self.create_subscription(Imu, '/esp32/imu', self.on_imu, 10)
        self.create_subscription(Float32MultiArray, '/esp32/diagnostics', self.on_diag, 10)

        # Republished on a steady timer, independent of when diagnostics or
        # IMU messages actually arrive, so a downstream staleness check on
        # /mission/active itself has something fresh to look at even if the
        # upstream feed briefly stalls.
        self.create_timer(0.1, self.publish_active_state)

        self.get_logger().info('heading_reference started - waiting for /esp32/diagnostics and /esp32/imu')

    def on_imu(self, msg: Imu):
        q = msg.orientation
        self.latest_rpy = quat_to_euler(q.w, q.x, q.y, q.z)
        self.publish_heading()

    def on_diag(self, msg: Float32MultiArray):
        d = msg.data
        # [loop_hz, sbus_ok, armed, autonomous_mode, estop_ok]. Tolerates the
        # older 3-field format (no autonomous_mode/estop_ok) by treating a
        # missing field as "not autonomous" / "not estop_ok" - i.e. failing
        # toward "mission not active", never guessing it into True.
        self._last_diag_time = time.monotonic()
        armed = len(d) > 2 and d[2] > 0.5
        autonomous_mode = len(d) > 3 and d[3] > 0.5
        estop_ok = (len(d) <= 4) or (d[4] > 0.5)

        self._armed = armed
        self._autonomous_mode = autonomous_mode and estop_ok

        mission_active = self._compute_mission_active()
        if mission_active and not self._prev_mission_active:
            self.reference_rpy = self.latest_rpy
            self.reset_stage_pub.publish(Empty())
            self.get_logger().info(
                'autonomous engaged - zeroing heading reference at '
                f'roll={math.degrees(self.reference_rpy[0]):.1f} '
                f'pitch={math.degrees(self.reference_rpy[1]):.1f} '
                f'yaw={math.degrees(self.reference_rpy[2]):.1f} deg (absolute, pre-zero)'
            )
        elif self._prev_mission_active and not mission_active:
            self.get_logger().info('autonomous disengaged')
        self._prev_mission_active = mission_active

    def _compute_mission_active(self) -> bool:
        if self._last_diag_time is None:
            return False
        if time.monotonic() - self._last_diag_time > DIAG_STALE_S:
            return False
        return self._armed and self._autonomous_mode

    def publish_heading(self):
        roll, pitch, yaw = self.latest_rpy
        if self.reference_rpy is not None:
            ref_roll, ref_pitch, ref_yaw = self.reference_rpy
            roll = wrap_pi(roll - ref_roll)
            pitch = wrap_pi(pitch - ref_pitch)
            yaw = wrap_pi(yaw - ref_yaw)
            zeroed = 1.0
        else:
            zeroed = 0.0
        msg = Float32MultiArray()
        msg.data = [roll, pitch, yaw, zeroed]
        self.heading_pub.publish(msg)

    def publish_active_state(self):
        # Re-evaluate staleness here too (not just in on_diag), so a link
        # that goes silent - rather than one that keeps sending armed=0 -
        # still drives /mission/active to False within DIAG_STALE_S.
        mission_active = self._compute_mission_active()
        if self._prev_mission_active and not mission_active:
            self.get_logger().warning('diagnostics went stale - forcing mission inactive')
        self._prev_mission_active = mission_active
        msg = Bool()
        msg.data = mission_active
        self.active_pub.publish(msg)


def main():
    rclpy.init()
    node = HeadingReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
