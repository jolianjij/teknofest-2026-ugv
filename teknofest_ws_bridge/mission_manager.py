#!/usr/bin/env python3
"""
Phase 1 autonomous mission: reactive lidar corridor-following, gated on
/mission/active (heading_reference.py's armed+autonomous+estop_ok latch),
plus an ExecuteStage action server so the deployed stage-sign pipeline
(ugv_stage_sign_detector: sign detector -> stage_manager -> stage action
dispatcher, see the ugv_stage_interfaces package) can track which numbered
stage the vehicle is currently in.

Scope, stated plainly: this node makes the vehicle drive down a corridor and
stop for obstacles, and it tracks which stage is active. It does NOT yet
give each stage (traffic cones, sliding doors, fire zone, ...) its own
driving behavior - every stage currently gets the same baseline corridor
following. Wiring in real per-stage behaviors (a Nav2 hand-off for cones, a
timed crossing for the door, aim+fire for the fire zone) is real, separate
follow-up work once this baseline is confirmed safe and correct on the
actual vehicle. Every stage's ExecuteStage goal is accepted and simply
tracked; it never changes what the drive loop below does.

Safety model (read this before changing the drive loop):
  - This node NEVER decides whether the vehicle is allowed to move by
    itself. It publishes /cmd_vel continuously; the ESP32 firmware only
    acts on /cmd_vel while its own state machine is in AUTONOMOUS (SwB
    switched to autonomous, armed, e-stop closed, not in aim mode, not
    braking) - see esp32_firmware.ino's loop(). Flipping back to manual,
    disarming, or hitting the e-stop regains control immediately at the
    firmware level, with no dependency on this node behaving correctly.
  - /mission/active (from heading_reference.py) is treated as stale after
    ACTIVE_STALE_S with no update, independently of whatever
    heading_reference.py itself decided - this is deliberate
    defense-in-depth, not redundant paranoia: three independent layers each
    fail toward "stopped" (ESP32 cmd_vel timeout -> this node's staleness
    check on /mission/active -> heading_reference's own staleness check on
    /esp32/diagnostics).
  - /scan is treated the same way: a stale or missing scan means the
    vehicle cannot safely judge distance to anything, so it stops instead
    of continuing on stale geometry.
  - All the corridor-following gains and distances (SAFETY_STOP_M,
    CAUTION_DISTANCE_M, K_LATERAL, K_HEADING, CRUISE_LINEAR, ...) plus the
    stall-detection/power-through constants live in mission_logic.py now
    (split out 2026-09-11 so that ROS-independent logic can be
    unit-tested without rclpy - see that file's own docstring) - they are
    reasoned-about placeholders, not measured/tuned values, and have never
    been validated against the real vehicle's actual braking distance,
    turning response, or lidar mounting geometry. Do not trust them at
    anything other than a very low speed_scale (see serial_bridge.py's
    /tuning/drive) until they've been tuned on the real course, per the
    project's hardware-validation policy.
"""
import asyncio
import threading
import time
import urllib.request

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from ugv_stage_interfaces.action import ExecuteStage

from mission_logic import (
    FORWARD_SECTOR_DEG,
    IMU_STALE_S,
    STALL_BOOST_LINEAR,
    SlidingDoorController,
    StallDetector,
    SteeringRateLimiter,
    TurnAdvanceController,
    compute_drive,
    min_range_in_sector,
    quat_to_pitch_deg,
)

ACTIVE_STALE_S = 0.5   # /mission/active republishes at 10Hz (heading_reference.py)
SCAN_STALE_S = 0.5     # /scan publishes at ~10Hz (rplidar_ros)
DOOR_STATE_STALE_S = 1.0   # /door/state publishes at camera rate (~20-30Hz) - see on_door_state's staleness fallback
DOOR_STAGE_BEHAVIOR = 'sliding_doors'  # must match stage_manager_node.py's DEFAULT_BEHAVIORS entry exactly
DRIVE_LOOP_HZ = 10.0

# Auto-record cameras+telemetry for the duration of the mission (explicit
# request: "record the camera feed in the mission"). Reuses the exact same
# start/stop the dashboard's own Record button calls (web_dashboard.py),
# rather than duplicating subprocess-management code here - this node just
# hits the same local HTTP endpoint the button does, on the mission-active
# rising/falling edge. Assumes the dashboard is running on localhost, which
# is how this whole stack is deployed (see CLAUDE.md's deployment table).
DASHBOARD_RECORD_URL = "http://localhost:8080/record/{action}"
RECORD_HTTP_TIMEOUT_S = 3.0

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.mission_active = False
        self._last_active_msg_time = 0.0
        self.heading_zeroed = False
        self.yaw_rel = 0.0
        self._last_scan = None
        self._last_scan_time = 0.0
        self._forward_accel = 0.0
        self._pitch_deg = 0.0
        self._last_imu_time = 0.0
        self._last_drive_loop_t = None
        self.stall_detector = StallDetector()
        self.steering_limiter = SteeringRateLimiter()
        self.turn_controller = TurnAdvanceController()
        self.door_controller = SlidingDoorController()

        self.current_stage_id = None
        self.current_behavior = None

        # /door/state (door_direction_node.py's DoorTracker output):
        # [direction, centroid_x_norm, velocity_px_s, blocked_fraction,
        #  period_s, clear_elapsed_s, blocked]. Defaults fail toward "don't
        # commit" - direction 0.0 (unknown) and blocked True are both
        # conditions SlidingDoorController refuses to commit on.
        self._door_direction = 0.0
        self._door_blocked = True
        self._door_period_s = 0.0
        self._door_clear_elapsed_s = 0.0
        self._last_door_state_time = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.behavior_pub = self.create_publisher(String, '/mission/current_behavior', 10)

        self.create_subscription(Bool, '/mission/active', self.on_active, 10)
        self.create_subscription(Float32MultiArray, '/mission/heading', self.on_heading, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Imu, '/esp32/imu', self.on_imu, 10)
        self.create_subscription(Float32MultiArray, '/door/state', self.on_door_state, 10)

        self.action_server = ActionServer(
            self,
            ExecuteStage,
            '/mission/execute_stage',
            execute_callback=self.execute_stage,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
        )

        self.create_timer(1.0 / DRIVE_LOOP_HZ, self.drive_loop)
        self.get_logger().info(
            'mission_manager started - Phase 1 (corridor-following + stage tracking, '
            'no per-stage behavior differentiation yet)'
        )

    # --- subscriptions ---

    def on_active(self, msg: Bool):
        new_active = bool(msg.data)
        if new_active != self.mission_active:
            action = "start" if new_active else "stop"
            # Fire-and-forget in a background thread - this is a local HTTP
            # call to the dashboard's own recording endpoint, but network
            # calls (even localhost ones) have no place blocking a ROS
            # callback that the drive loop's safety timing depends on.
            threading.Thread(target=self._request_recording, args=(action,), daemon=True).start()
        self.mission_active = new_active
        self._last_active_msg_time = time.monotonic()

    def _request_recording(self, action: str):
        url = DASHBOARD_RECORD_URL.format(action=action)
        try:
            with urllib.request.urlopen(url, timeout=RECORD_HTTP_TIMEOUT_S) as resp:
                resp.read()
            self.get_logger().info(f'mission {action} - requested recording {action}')
        except Exception as e:
            # Not fatal to the mission either way - a failed recording
            # request must never be allowed to affect driving.
            self.get_logger().warning(f'could not {action} recording via dashboard: {e}')

    def on_heading(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 4:
            return
        self.yaw_rel = d[2]
        self.heading_zeroed = d[3] > 0.5

    def on_scan(self, msg: LaserScan):
        self._last_scan = msg
        self._last_scan_time = time.monotonic()

    def on_door_state(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 6:
            return
        self._door_direction = d[0]
        self._door_period_s = d[4]
        self._door_clear_elapsed_s = d[5]
        # index 6 ('blocked') is additive - fall back to the fail-safe
        # default (treat as blocked, refuse to commit) if an older
        # door_direction_node build is still running on the Pi.
        self._door_blocked = bool(d[6] > 0.5) if len(d) > 6 else True
        self._last_door_state_time = time.monotonic()

    def on_imu(self, msg: Imu):
        # IMU_FORWARD_AXIS assumption (see module comment above
        # StallDetector): +X is forward. Fix here if the real mounting
        # disagrees.
        self._forward_accel = msg.linear_acceleration.x
        q = msg.orientation
        self._pitch_deg = quat_to_pitch_deg(q.w, q.x, q.y, q.z)
        self._last_imu_time = time.monotonic()

    # --- ExecuteStage action server ---

    def on_goal(self, goal_request: ExecuteStage.Goal) -> GoalResponse:
        if not goal_request.behavior:
            self.get_logger().warning('rejecting a stage goal with an empty behavior')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def on_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_stage(self, goal_handle):
        stage_id = goal_handle.request.stage_id
        behavior = goal_handle.request.behavior
        self.current_stage_id = stage_id
        self.current_behavior = behavior
        if behavior == DOOR_STAGE_BEHAVIOR:
            # Fresh state machine for every attempt at this stage - a stale
            # 'commit'/'cleared' left over from an earlier engagement must
            # never leak into a new one.
            self.door_controller.reset()
            self.get_logger().info(f'stage {stage_id} ("{behavior}") is now active - door commit logic engaged')
        else:
            self.get_logger().info(
                f'stage {stage_id} ("{behavior}") is now active - Phase 1 baseline corridor-'
                f'following continues unchanged; no behavior-specific driving is implemented yet'
            )
        feedback = ExecuteStage.Feedback()
        result = ExecuteStage.Result()
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = True
                    result.message = f'stage {stage_id} superseded by the next stage transition'
                    return result
                if not self.mission_active:
                    goal_handle.abort()
                    result.success = False
                    result.message = f'stage {stage_id} aborted - mission is not active'
                    return result
                feedback.progress = 0.0  # no per-stage completion metric exists yet (Phase 1)
                feedback.state = f'corridor_following:{behavior}'
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.2)
        finally:
            if self.current_stage_id == stage_id:
                self.current_stage_id = None
                self.current_behavior = None

        goal_handle.abort()
        result.success = False
        result.message = 'node shutting down'
        return result

    # --- drive loop ---

    def drive_loop(self):
        now = time.monotonic()
        active = self.mission_active and (now - self._last_active_msg_time) < ACTIVE_STALE_S
        scan_fresh = self._last_scan is not None and (now - self._last_scan_time) < SCAN_STALE_S

        # dt for the StallDetector/SteeringRateLimiter: measured between
        # actual calls, not assumed from DRIVE_LOOP_HZ - a late/skipped
        # timer tick would otherwise make either think less time passed
        # than really did.
        dt = (now - self._last_drive_loop_t) if self._last_drive_loop_t is not None else (1.0 / DRIVE_LOOP_HZ)
        self._last_drive_loop_t = now

        if not active:
            self._stop()
            self._publish_state('idle')
            self.steering_limiter.reset()
            self.turn_controller.reset()
            # A genuine mission stop (disarm/re-arm, not just a momentary
            # scan/heading blip - those branches below don't reset this)
            # must not leak a stale 'commit'/'cleared' door state into the
            # next engagement.
            self.door_controller.reset()
        elif not self.heading_zeroed:
            # Should not normally happen (heading_reference zeroes in the same
            # step it flips mission_active) but if it ever does, don't drive
            # on an undefined heading reference.
            self._stop()
            self._publish_state('stopped')
            self.steering_limiter.reset()
            self.turn_controller.reset()
        elif not scan_fresh:
            self._stop()
            self._publish_state('stopped')
            self.get_logger().warning('lidar scan stale - stopping')
            self.steering_limiter.reset()
            self.turn_controller.reset()
        else:
            linear, angular, mode = compute_drive(self._last_scan, self.yaw_rel, self._pitch_deg)

            # Sliding-door commit logic (Stage 6 only) - see
            # SlidingDoorController's own docstring. Only consulted when
            # compute_drive() itself isn't already demanding a hard_stop
            # (a genuine close-range front obstacle always wins, whether
            # that's the door itself closing early or something unrelated -
            # the door controller's internal timers simply pause and resume
            # next tick once the path clears again, since a skipped tick
            # here does not advance its dt). Returns None while the door
            # stage isn't close enough yet ('approach') or has already been
            # crossed ('cleared') - baseline compute_drive() output applies
            # in both of those cases exactly as it would for any other
            # stage.
            door_result = None
            if self.current_behavior == DOOR_STAGE_BEHAVIOR and mode != 'hard_stop':
                door_fresh = (now - self._last_door_state_time) < DOOR_STATE_STALE_S
                door_result = self.door_controller.update(
                    self._last_scan,
                    min_range_in_sector(self._last_scan, 0.0, FORWARD_SECTOR_DEG),
                    self._door_direction if door_fresh else 0.0,
                    self._door_blocked if door_fresh else True,
                    self._door_period_s if door_fresh else 0.0,
                    self._door_clear_elapsed_s if door_fresh else 0.0,
                    dt,
                )

            if door_result is not None:
                linear, angular, mode = door_result
                # Deliberate, already-final values (stopped-and-observing,
                # or a timed commit with its own right-wall-hug steering) -
                # not something to re-smooth or reinterpret through the
                # turn/steering-rate machinery below.
                self.steering_limiter.reset()
                self.turn_controller.reset()
            elif mode in ('hard_stop', 'side_contact_risk'):
                # Stop turning too, right now - not something to ease into
                # an ALIGN phase for, and nothing to smooth. side_contact_risk
                # carries its own escape-steer angular value (see
                # SIDE_CONTACT_STEER in mission_logic.py) which must reach
                # the wheels as computed - passing it through
                # turn_controller.update() here would clamp it down to
                # ADVANCE_ANGULAR_TRIM_MAX (0.15), far too weak for what is
                # meant to be an emergency escape turn.
                self.steering_limiter.reset()
                self.turn_controller.reset()
            else:
                # Turn-then-advance (2026-09-11, see mission_logic.py's own
                # comment above TurnAdvanceController for the full
                # reasoning): a real turn stops forward motion and commits
                # to a decisive, unsmoothed rotation; a small trim
                # correction during normal advancing still goes through the
                # slew-rate limiter so it stays smooth.
                angular = self.turn_controller.update(angular)
                if self.turn_controller.aligning:
                    linear = 0.0
                    mode = mode + '_align'
                    self.steering_limiter.reset()
                else:
                    angular = self.steering_limiter.update(angular, dt)

            imu_fresh = (now - self._last_imu_time) < IMU_STALE_S
            if imu_fresh:
                boosting = self.stall_detector.update(linear, self._forward_accel, dt)
                if boosting:
                    linear = STALL_BOOST_LINEAR if linear >= 0.0 else -STALL_BOOST_LINEAR
                    mode = 'stall_boost'
                elif self.stall_detector.in_cooldown and mode not in ('hard_stop',):
                    # Actually still within the post-give-up cooldown window
                    # (in_cooldown clears itself once that timer expires,
                    # unlike gave_up which is a one-shot event flag) -
                    # report it distinctly so it's visible on the dashboard,
                    # but still drive/steer normally (compute_drive already
                    # decided linear/angular for this tick; the cooldown
                    # just blocks another boost attempt).
                    mode = mode + '_stall_cooldown'
            # No IMU signal -> no stall detection at all, fail toward the
            # normal (unboosted) compute_drive() behavior rather than guess.

            self._publish_cmd_vel(linear, angular)
            self._publish_state(f'running:{mode}')

        behavior_msg = String()
        behavior_msg.data = self.current_behavior or ''
        self.behavior_pub.publish(behavior_msg)

    def _stop(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _publish_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.action_server.destroy()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
