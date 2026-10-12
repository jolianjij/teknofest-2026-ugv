"""
Regression tests for mission_logic.py's corridor-following and
stall-detection/obstacle-power-through logic (2026-09-11) - both are
deliberately ROS-independent (see that file's own docstring) so they can be
tested exactly like this, without a robot, a lidar, or the Orange Pi.

These exist because tonight's changes had zero prior test coverage: the
stall-detection state machine is new, and compute_drive()/min_range_in_sector()
had never been exercised against anything before either. Run via:
pytest test_mission_logic.py
"""
import math

import pytest

from mission_logic import (
    ADVANCE_ANGULAR_TRIM_MAX,
    ALIGN_ANGULAR_MAX,
    CAUTION_DISTANCE_M,
    CLIMB_BOOST_MAX,
    CLIMB_PITCH_FULL_BOOST_DEG,
    CORNER_TURN_BOOST_MAX,
    CRUISE_LINEAR,
    DOOR_COMMIT_LINEAR,
    DOOR_DIRECTION_RIGHT,
    DOOR_OBSERVE_TIMEOUT_S,
    DOOR_RIGHT_HUG_SETPOINT_M,
    DOOR_STANDOFF_M,
    DOOR_TRANSIT_SAFETY_FACTOR,
    DOOR_TRANSIT_TIME_S,
    HEADING_DEADZONE_RAD,
    LIDAR_FRONT_OFFSET_RAD,
    MAX_ANGULAR,
    MAX_ANGULAR_SLEW_PER_S,
    MIN_CYCLES_OBSERVED,
    PITCH_CLIMB_SIGN,
    SAFETY_STOP_M,
    SIDE_CONTACT_MARGIN_M,
    SIDE_CONTACT_STEER,
    STALL_ACCEL_THRESHOLD_MPS2,
    STALL_BOOST_MAX_S,
    STALL_COOLDOWN_S,
    STALL_DETECT_S,
    STALL_MIN_COMMAND,
    TURN_ALIGN_ENTER,
    TURN_ALIGN_EXIT,
    WALL_ERROR_DEADZONE_M,
    SlidingDoorController,
    SteeringRateLimiter,
    StallDetector,
    TurnAdvanceController,
    compute_drive,
    min_range_in_sector,
    quat_to_pitch_deg,
)


class FakeScan:
    """Duck-typed stand-in for sensor_msgs/LaserScan - only the fields
    min_range_in_sector()/compute_drive() actually read."""

    def __init__(self, ranges, angle_min=-math.pi, angle_max=math.pi,
                 range_min=0.1, range_max=12.0):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = (angle_max - angle_min) / max(1, len(ranges) - 1)
        self.range_min = range_min
        self.range_max = range_max


def _scan_with_range_at(vehicle_angle_deg, distance_m, n=720, background=float('inf')):
    """Builds a scan where the closest point at vehicle_angle_deg (0=front,
    +90=left, -90=right, post LIDAR_FRONT_OFFSET_RAD correction) reads
    distance_m. Everything else is `background`."""
    ranges = [background] * n
    angle_min, angle_max = -math.pi, math.pi
    angle_increment = (angle_max - angle_min) / (n - 1)
    # Raw scan angle (pre-offset) that lands on vehicle_angle_deg once
    # LIDAR_FRONT_OFFSET_RAD is added inside min_range_in_sector.
    target_angle = math.radians(vehicle_angle_deg) - LIDAR_FRONT_OFFSET_RAD
    target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
    idx = int(round((target_angle - angle_min) / angle_increment))
    idx = max(0, min(n - 1, idx))
    ranges[idx] = distance_m
    return FakeScan(ranges, angle_min, angle_max)


# ---------- min_range_in_sector ----------

def test_min_range_finds_closest_point_dead_ahead():
    scan = _scan_with_range_at(0.0, 1.2)
    assert min_range_in_sector(scan, 0.0, 20.0) == pytest.approx(1.2, abs=0.05)


def test_min_range_ignores_points_outside_sector():
    scan = _scan_with_range_at(90.0, 0.5)  # a close point on the LEFT only
    # The forward sector shouldn't see it.
    assert min_range_in_sector(scan, 0.0, 20.0) == float('inf')


def test_min_range_empty_scan_returns_inf():
    scan = FakeScan([])
    assert min_range_in_sector(scan, 0.0, 20.0) == float('inf')


def test_min_range_rejects_out_of_hardware_range_readings():
    scan = _scan_with_range_at(0.0, 50.0)  # beyond range_max=12.0
    assert min_range_in_sector(scan, 0.0, 20.0) == float('inf')


# ---------- compute_drive: hard stop ----------

def test_hard_stop_inside_safety_distance():
    scan = _scan_with_range_at(0.0, SAFETY_STOP_M - 0.1)
    linear, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert (linear, angular, mode) == (0.0, 0.0, 'hard_stop')


def test_no_hard_stop_just_outside_safety_distance():
    scan = _scan_with_range_at(0.0, SAFETY_STOP_M + 0.05)
    _, _, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode != 'hard_stop'


# ---------- compute_drive: wall following / centering ----------

def test_centers_between_two_walls():
    # Left wall closer than right -> should steer away from the closer
    # (left) wall, i.e. turn right (negative angular), per the module's own
    # convention (left>right -> turn left away from right wall, so
    # left<right -> turn right away from left wall). Both distances stay
    # above SIDE_CONTACT_MARGIN_M so this exercises normal centering, not
    # the 2026-09-11 side-contact emergency guard (see that test section).
    ranges = [float('inf')] * 720
    scan = FakeScan(ranges)
    scan_l = _scan_with_range_at(90.0, SIDE_CONTACT_MARGIN_M + 0.15)
    scan_r = _scan_with_range_at(-90.0, 2.0)
    # Combine both into one scan (both distinct indices, background stays inf)
    combined = FakeScan([float('inf')] * 720)
    combined.ranges = list(scan_l.ranges)
    for i, r in enumerate(scan_r.ranges):
        if r != float('inf'):
            combined.ranges[i] = r
    linear, angular, mode = compute_drive(combined, yaw_rel=0.0)
    assert mode == 'center'
    assert angular < 0.0  # closer wall is on the left -> turn right (away)
    assert linear > 0.0


def test_follows_single_left_wall_too_close_steers_away():
    # Closer than WALL_FOLLOW_SETPOINT_M but still above
    # SIDE_CONTACT_MARGIN_M - this tests the ordinary follow_left response,
    # not the 2026-09-11 side-contact emergency guard.
    scan = _scan_with_range_at(90.0, SIDE_CONTACT_MARGIN_M + 0.1)
    _, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'follow_left'
    assert angular < 0.0  # too close on the left -> turn right


def test_tiny_wall_error_within_deadzone_produces_no_steering():
    # Regression (2026-09-11): noise-sized wall-distance differences must
    # not produce any steering command at all - this is what was causing
    # oscillation across zero on one wheel (see motor_mixer.cpp's matching
    # fix). Symmetric distances well inside WALL_ERROR_DEADZONE_M.
    scan_l = _scan_with_range_at(90.0, 1.0)
    scan_r = _scan_with_range_at(-90.0, 1.0 + WALL_ERROR_DEADZONE_M * 0.5)
    combined = FakeScan(list(scan_l.ranges))
    for i, r in enumerate(scan_r.ranges):
        if r != float('inf'):
            combined.ranges[i] = r
    _, angular, mode = compute_drive(combined, yaw_rel=0.0)
    assert mode == 'center'
    assert angular == pytest.approx(0.0)


def test_wall_error_beyond_deadzone_still_steers():
    # Stays above SIDE_CONTACT_MARGIN_M - see test_centers_between_two_walls.
    scan_l = _scan_with_range_at(90.0, SIDE_CONTACT_MARGIN_M + 0.15)
    scan_r = _scan_with_range_at(-90.0, 2.0)
    combined = FakeScan(list(scan_l.ranges))
    for i, r in enumerate(scan_r.ranges):
        if r != float('inf'):
            combined.ranges[i] = r
    _, angular, mode = compute_drive(combined, yaw_rel=0.0)
    assert mode == 'center'
    assert angular < -0.05  # clearly still steers away from the closer (left) wall


def test_tiny_heading_drift_within_deadzone_produces_no_correction():
    scan = FakeScan([float('inf')] * 720)
    _, angular, _ = compute_drive(scan, yaw_rel=HEADING_DEADZONE_RAD * 0.5)
    assert angular == pytest.approx(0.0)


def test_heading_drift_beyond_deadzone_still_corrects():
    scan = FakeScan([float('inf')] * 720)
    _, angular, _ = compute_drive(scan, yaw_rel=HEADING_DEADZONE_RAD * 5)
    assert angular < 0.0


def test_open_space_no_walls_goes_straight():
    scan = FakeScan([float('inf')] * 720)
    linear, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'open'
    assert angular == pytest.approx(0.0)
    assert linear == pytest.approx(CRUISE_LINEAR)


def test_heading_drift_correction_applies_in_open_space():
    scan = FakeScan([float('inf')] * 720)
    _, angular_left_drift, _ = compute_drive(scan, yaw_rel=0.5)   # drifted left
    _, angular_right_drift, _ = compute_drive(scan, yaw_rel=-0.5)  # drifted right
    assert angular_left_drift < 0.0   # correct back right
    assert angular_right_drift > 0.0  # correct back left


def test_angular_is_clamped():
    scan = _scan_with_range_at(90.0, 0.01)  # extreme close-left case
    _, angular, _ = compute_drive(scan, yaw_rel=10.0)  # huge heading error too
    assert abs(angular) <= MAX_ANGULAR + 1e-6


def test_speed_scales_down_approaching_obstacle():
    near = _scan_with_range_at(0.0, SAFETY_STOP_M + 0.05)
    mid = _scan_with_range_at(0.0, (SAFETY_STOP_M + CAUTION_DISTANCE_M) / 2)
    far = _scan_with_range_at(0.0, CAUTION_DISTANCE_M + 1.0)
    lin_near, _, _ = compute_drive(near, yaw_rel=0.0)
    lin_mid, _, _ = compute_drive(mid, yaw_rel=0.0)
    lin_far, _, _ = compute_drive(far, yaw_rel=0.0)
    assert lin_near < lin_mid < lin_far
    assert lin_far == pytest.approx(CRUISE_LINEAR)


def _combined_scan(front_m=None, left_m=None, right_m=None):
    """Combine independent front/left/right readings into one scan - each
    optional, an omitted direction stays at background (open space)."""
    combined = FakeScan([float('inf')] * 720)
    for angle_deg, distance in ((0.0, front_m), (90.0, left_m), (-90.0, right_m)):
        if distance is None:
            continue
        partial = _scan_with_range_at(angle_deg, distance)
        for i, r in enumerate(partial.ranges):
            if r != float('inf'):
                combined.ranges[i] = r
    return combined


def test_corner_boost_amplifies_turn_near_a_wall():
    # Same left/right asymmetry (0.3m, small enough neither case saturates
    # MAX_ANGULAR) in both cases - only the forward distance differs (far
    # vs. right at the edge of SAFETY_STOP_M). The turn should be
    # noticeably stronger when close to the wall ahead - this is the actual
    # "guaranteed turn" mechanism, not just a smaller speed.
    far_scan = _combined_scan(front_m=CAUTION_DISTANCE_M + 1.0, left_m=1.0, right_m=1.3)
    near_scan = _combined_scan(front_m=SAFETY_STOP_M + 0.05, left_m=1.0, right_m=1.3)
    _, angular_far, mode_far = compute_drive(far_scan, yaw_rel=0.0)
    _, angular_near, mode_near = compute_drive(near_scan, yaw_rel=0.0)
    assert mode_far == mode_near == 'center'
    assert angular_far < 0.0
    assert angular_near < 0.0
    assert abs(angular_near) > abs(angular_far)


def test_corner_boost_does_not_invent_a_turn_in_open_space():
    far_scan = _combined_scan(front_m=CAUTION_DISTANCE_M + 1.0)
    near_scan = _combined_scan(front_m=SAFETY_STOP_M + 0.05)
    _, angular_far, mode_far = compute_drive(far_scan, yaw_rel=0.0)
    _, angular_near, mode_near = compute_drive(near_scan, yaw_rel=0.0)
    assert mode_far == mode_near == 'open'
    assert angular_far == pytest.approx(0.0)
    assert angular_near == pytest.approx(0.0)


def test_corner_boost_matches_expected_multiplier_near_the_wall_edge():
    from mission_logic import K_LATERAL, WALL_ERROR_DEADZONE_M as DZ
    scan = _combined_scan(front_m=SAFETY_STOP_M + 0.001, left_m=1.0, right_m=1.3)
    _, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'center'
    raw_error = 1.0 - 1.3  # left - right
    deadzoned_error = raw_error + DZ  # raw_error is negative and beyond the deadzone
    expected_unboosted = K_LATERAL * deadzoned_error
    speed_scale = 0.001 / (CAUTION_DISTANCE_M - SAFETY_STOP_M)
    expected_boost = 1.0 + (1.0 - speed_scale) * CORNER_TURN_BOOST_MAX
    assert angular == pytest.approx(expected_unboosted * expected_boost, rel=0.05)


# ---------- StallDetector ----------

DT = 0.1  # matches DRIVE_LOOP_HZ=10 in mission_manager.py


def test_no_stall_while_actually_accelerating():
    sd = StallDetector()
    boosting = False
    for _ in range(50):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=1.0, dt=DT)
    assert boosting is False


def test_no_stall_while_not_trying_to_move():
    sd = StallDetector()
    boosting = False
    for _ in range(50):
        boosting = sd.update(commanded_linear=0.0, forward_accel_mps2=0.0, dt=DT)
    assert boosting is False


def test_stall_detected_after_threshold_duration():
    sd = StallDetector()
    boosting = False
    ticks = int(STALL_DETECT_S / DT) + 2
    for _ in range(ticks):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert boosting is True


def test_no_stall_before_threshold_duration():
    sd = StallDetector()
    ticks = int(STALL_DETECT_S / DT) - 2
    boosting = False
    for _ in range(max(1, ticks)):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert boosting is False


def test_boost_stops_once_acceleration_resumes():
    sd = StallDetector()
    ticks = int(STALL_DETECT_S / DT) + 2
    for _ in range(ticks):
        sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert sd.boosting is True
    # Made it through the obstacle - acceleration picks back up.
    boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=2.0, dt=DT)
    assert boosting is False
    assert sd.boosting is False


def test_boost_gives_up_after_max_boost_duration():
    sd = StallDetector()
    ticks_to_stall = int(STALL_DETECT_S / DT) + 2
    for _ in range(ticks_to_stall):
        sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert sd.boosting is True
    # Keep commanding forward with zero accel through the whole boost window
    # and past it - should give up, not boost forever.
    ticks_boost = int(STALL_BOOST_MAX_S / DT) + 2
    boosting = True
    for _ in range(ticks_boost):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert boosting is False
    assert sd.gave_up is True


def test_no_reattempt_during_cooldown_after_giving_up():
    sd = StallDetector()
    ticks_to_stall = int(STALL_DETECT_S / DT) + 2
    ticks_boost = int(STALL_BOOST_MAX_S / DT) + 2
    for _ in range(ticks_to_stall + ticks_boost):
        sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert sd.gave_up is True
    # Immediately keep trying through the cooldown window - must not boost again.
    ticks_cooldown = int(STALL_COOLDOWN_S / DT) - 2
    for _ in range(max(1, ticks_cooldown)):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
        assert boosting is False


def test_in_cooldown_clears_once_timer_expires_even_though_gave_up_stays_true():
    # Regression: gave_up is a one-shot event flag and stays True forever
    # after the first give-up until the next stall attempt - using it
    # directly for "are we currently in cooldown" status reporting would
    # wrongly report cooldown forever. in_cooldown must clear itself.
    sd = StallDetector()
    ticks_to_stall = int(STALL_DETECT_S / DT) + 2
    ticks_boost = int(STALL_BOOST_MAX_S / DT) + 2
    for _ in range(ticks_to_stall + ticks_boost):
        sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert sd.gave_up is True
    assert sd.in_cooldown is True
    # Let the cooldown fully elapse, but with no further attempt to move
    # (so nothing re-triggers boosting/gave_up).
    ticks_cooldown = int(STALL_COOLDOWN_S / DT) + 5
    for _ in range(ticks_cooldown):
        sd.update(commanded_linear=0.0, forward_accel_mps2=0.0, dt=DT)
    assert sd.gave_up is True       # still a stale one-shot flag - expected
    assert sd.in_cooldown is False  # but the actual cooldown has expired


def test_can_reattempt_after_cooldown_expires():
    sd = StallDetector()
    ticks_to_stall = int(STALL_DETECT_S / DT) + 2
    ticks_boost = int(STALL_BOOST_MAX_S / DT) + 2
    ticks_cooldown = int(STALL_COOLDOWN_S / DT) + 2
    for _ in range(ticks_to_stall + ticks_boost + ticks_cooldown):
        sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    # Cooldown has now fully elapsed - stalling again should be allowed to
    # trigger a fresh boost attempt.
    boosting = False
    for _ in range(ticks_to_stall):
        boosting = sd.update(commanded_linear=0.5, forward_accel_mps2=0.0, dt=DT)
    assert boosting is True


def test_stall_command_below_min_threshold_never_triggers():
    sd = StallDetector()
    boosting = False
    for _ in range(200):
        boosting = sd.update(commanded_linear=STALL_MIN_COMMAND - 0.01,
                              forward_accel_mps2=0.0, dt=DT)
    assert boosting is False


def test_moderate_acceleration_above_threshold_prevents_stall():
    sd = StallDetector()
    boosting = False
    for _ in range(200):
        boosting = sd.update(commanded_linear=0.5,
                              forward_accel_mps2=STALL_ACCEL_THRESHOLD_MPS2 + 0.05,
                              dt=DT)
    assert boosting is False


# ---------- SteeringRateLimiter ----------

def test_rate_limiter_starts_at_zero():
    rl = SteeringRateLimiter()
    assert rl.value == 0.0


def test_rate_limiter_caps_a_big_sudden_jump():
    rl = SteeringRateLimiter()
    out = rl.update(target_angular=MAX_ANGULAR, dt=DT)
    max_step = MAX_ANGULAR_SLEW_PER_S * DT
    assert out == pytest.approx(max_step)
    assert out < MAX_ANGULAR


def test_rate_limiter_converges_to_target_over_time():
    rl = SteeringRateLimiter()
    out = 0.0
    for _ in range(200):
        out = rl.update(target_angular=MAX_ANGULAR, dt=DT)
    assert out == pytest.approx(MAX_ANGULAR, abs=1e-6)


def test_rate_limiter_does_not_overshoot_a_small_target():
    rl = SteeringRateLimiter()
    small_target = 0.01
    out = rl.update(target_angular=small_target, dt=DT)
    assert out == pytest.approx(small_target)  # smaller than one max_step, reached directly


def test_rate_limiter_tracks_a_reversing_target_smoothly():
    rl = SteeringRateLimiter()
    for _ in range(200):
        rl.update(target_angular=MAX_ANGULAR, dt=DT)
    assert rl.value == pytest.approx(MAX_ANGULAR, abs=1e-6)
    out = rl.update(target_angular=-MAX_ANGULAR, dt=DT)
    max_step = MAX_ANGULAR_SLEW_PER_S * DT
    assert out == pytest.approx(MAX_ANGULAR - max_step)
    assert out > -MAX_ANGULAR  # did not jump straight to the new target


def test_rate_limiter_reset_returns_to_zero():
    rl = SteeringRateLimiter()
    for _ in range(200):
        rl.update(target_angular=MAX_ANGULAR, dt=DT)
    rl.reset()
    assert rl.value == 0.0


# ---------- quat_to_pitch_deg ----------

def _pitch_quat(pitch_deg):
    """Pure pitch (rotation about Y) quaternion for a given angle."""
    half = math.radians(pitch_deg) / 2.0
    return dict(w=math.cos(half), x=0.0, y=math.sin(half), z=0.0)


def test_pitch_identity_quaternion_is_zero():
    assert quat_to_pitch_deg(1.0, 0.0, 0.0, 0.0) == pytest.approx(0.0, abs=1e-9)


def test_pitch_positive_rotation():
    q = _pitch_quat(30.0)
    assert quat_to_pitch_deg(q['w'], q['x'], q['y'], q['z']) == pytest.approx(30.0, abs=1e-6)


def test_pitch_negative_rotation():
    q = _pitch_quat(-20.0)
    assert quat_to_pitch_deg(q['w'], q['x'], q['y'], q['z']) == pytest.approx(-20.0, abs=1e-6)


# ---------- climb boost in compute_drive ----------

def test_no_climb_boost_at_zero_pitch():
    scan = FakeScan([float('inf')] * 720)
    linear_flat, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=0.0)
    assert linear_flat == pytest.approx(CRUISE_LINEAR)


def test_climb_boost_increases_linear_speed():
    scan = FakeScan([float('inf')] * 720)
    climb_pitch = PITCH_CLIMB_SIGN * (CLIMB_PITCH_FULL_BOOST_DEG / 2.0)
    linear_flat, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=0.0)
    linear_climbing, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=climb_pitch)
    assert linear_climbing > linear_flat


def test_climb_boost_caps_at_max():
    scan = FakeScan([float('inf')] * 720)
    climb_pitch = PITCH_CLIMB_SIGN * (CLIMB_PITCH_FULL_BOOST_DEG * 5)  # well past full-boost angle
    linear, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=climb_pitch)
    expected_max = min(1.0, CRUISE_LINEAR * (1.0 + CLIMB_BOOST_MAX))
    assert linear == pytest.approx(expected_max)


def test_descending_pitch_does_not_reduce_speed():
    # Downhill (opposite sign from climbing) should never get LESS power
    # than flat ground - only ever boosted, never penalized.
    scan = FakeScan([float('inf')] * 720)
    descending_pitch = -PITCH_CLIMB_SIGN * (CLIMB_PITCH_FULL_BOOST_DEG / 2.0)
    linear_flat, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=0.0)
    linear_descending, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=descending_pitch)
    assert linear_descending == pytest.approx(linear_flat)


def test_climb_boost_never_exceeds_normalized_range():
    scan = FakeScan([float('inf')] * 720)
    linear, _, _ = compute_drive(scan, yaw_rel=0.0, pitch_deg=PITCH_CLIMB_SIGN * 1000.0)
    assert 0.0 <= linear <= 1.0


# ---------- TurnAdvanceController ----------

def test_turn_controller_starts_advancing():
    tc = TurnAdvanceController()
    assert tc.aligning is False


def test_small_angular_stays_advancing():
    tc = TurnAdvanceController()
    small = ADVANCE_ANGULAR_TRIM_MAX * 0.5  # comfortably under both the trim cap and the enter threshold
    out = tc.update(small)
    assert tc.aligning is False
    assert out == pytest.approx(small)  # within ADVANCE_ANGULAR_TRIM_MAX, passes through unclamped


def test_advance_trim_is_clamped():
    tc = TurnAdvanceController()
    out = tc.update(TURN_ALIGN_ENTER * 0.9)  # still below enter threshold, but bigger than the trim cap
    assert tc.aligning is False
    assert out == pytest.approx(ADVANCE_ANGULAR_TRIM_MAX)


def test_large_angular_triggers_align():
    tc = TurnAdvanceController()
    big = TURN_ALIGN_ENTER * 2.0
    out = tc.update(big)
    assert tc.aligning is True
    assert out == pytest.approx(ALIGN_ANGULAR_MAX)


def test_align_negative_direction_clamped_correctly():
    tc = TurnAdvanceController()
    out = tc.update(-TURN_ALIGN_ENTER * 2.0)
    assert tc.aligning is True
    assert out == pytest.approx(-ALIGN_ANGULAR_MAX)


def test_align_persists_through_hysteresis_band():
    # Once aligning, a value between EXIT and ENTER thresholds should NOT
    # immediately drop back to advancing - only below EXIT does.
    tc = TurnAdvanceController()
    tc.update(TURN_ALIGN_ENTER * 2.0)  # enter align
    assert tc.aligning is True
    mid_band = (TURN_ALIGN_ENTER + TURN_ALIGN_EXIT) / 2.0
    tc.update(mid_band)
    assert tc.aligning is True  # still above EXIT, stays aligning


def test_align_exits_below_exit_threshold():
    tc = TurnAdvanceController()
    tc.update(TURN_ALIGN_ENTER * 2.0)  # enter align
    assert tc.aligning is True
    tc.update(TURN_ALIGN_EXIT * 0.5)
    assert tc.aligning is False


def test_advance_does_not_enter_align_below_enter_threshold():
    tc = TurnAdvanceController()
    tc.update(TURN_ALIGN_ENTER * 0.99)
    assert tc.aligning is False


def test_turn_controller_reset():
    tc = TurnAdvanceController()
    tc.update(TURN_ALIGN_ENTER * 2.0)
    assert tc.aligning is True
    tc.reset()
    assert tc.aligning is False


# ---------- side-contact guard ----------

def _scan_with_two_ranges_at(pair_a, pair_b, n=720, background=float('inf')):
    """Like _scan_with_range_at but places two close points - pair_a and
    pair_b are (vehicle_angle_deg, distance_m) tuples."""
    ranges = [background] * n
    angle_min, angle_max = -math.pi, math.pi
    angle_increment = (angle_max - angle_min) / (n - 1)
    for vehicle_angle_deg, distance_m in (pair_a, pair_b):
        target_angle = math.radians(vehicle_angle_deg) - LIDAR_FRONT_OFFSET_RAD
        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
        idx = int(round((target_angle - angle_min) / angle_increment))
        idx = max(0, min(n - 1, idx))
        ranges[idx] = distance_m
    return FakeScan(ranges, angle_min, angle_max)


def test_side_contact_left_steers_right_and_stops():
    scan = _scan_with_range_at(90.0, SIDE_CONTACT_MARGIN_M - 0.05)
    linear, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'side_contact_risk'
    assert linear == 0.0
    assert angular == pytest.approx(-SIDE_CONTACT_STEER)


def test_side_contact_right_steers_left_and_stops():
    scan = _scan_with_range_at(-90.0, SIDE_CONTACT_MARGIN_M - 0.05)
    linear, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'side_contact_risk'
    assert linear == 0.0
    assert angular == pytest.approx(SIDE_CONTACT_STEER)


def test_side_contact_both_critical_holds_heading():
    scan = _scan_with_two_ranges_at(
        (90.0, SIDE_CONTACT_MARGIN_M - 0.05),
        (-90.0, SIDE_CONTACT_MARGIN_M - 0.05),
    )
    linear, angular, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'side_contact_risk'
    assert linear == 0.0
    assert angular == 0.0


def test_no_side_contact_just_outside_margin():
    scan = _scan_with_range_at(90.0, SIDE_CONTACT_MARGIN_M + 0.05)
    _, _, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode != 'side_contact_risk'


def test_normal_wall_following_unaffected_by_side_contact_guard():
    # Comfortably beyond SIDE_CONTACT_MARGIN_M on both sides - should behave
    # exactly like the existing centering test, not trip the new guard.
    scan = _scan_with_two_ranges_at((90.0, 1.0), (-90.0, 1.5))
    _, _, mode = compute_drive(scan, yaw_rel=0.0)
    assert mode == 'center'


# ---------- SlidingDoorController ----------

def _open_scan(forward_m=10.0):
    return _scan_with_range_at(0.0, forward_m)


def test_door_controller_stays_in_approach_until_standoff():
    dc = SlidingDoorController()
    result = dc.update(_open_scan(DOOR_STANDOFF_M + 0.5), DOOR_STANDOFF_M + 0.5,
                        door_direction=0.0, door_blocked=True, door_period_s=0.0,
                        door_clear_elapsed_s=0.0, dt=0.1)
    assert result is None
    assert dc.state == 'approach'


def test_door_controller_enters_observe_at_standoff():
    dc = SlidingDoorController()
    result = dc.update(_open_scan(DOOR_STANDOFF_M - 0.1), DOOR_STANDOFF_M - 0.1,
                        door_direction=0.0, door_blocked=True, door_period_s=0.0,
                        door_clear_elapsed_s=0.0, dt=0.1)
    assert result == (0.0, 0.0, 'door_observe')
    assert dc.state == 'observe'


def _run_to_wait_window(dc, scan, forward_m, cycles=MIN_CYCLES_OBSERVED):
    """Drives the controller from 'observe' through `cycles` full
    blocked->clear transitions, ending on a clear reading so callers can
    then test the commit decision itself."""
    dc.update(scan, forward_m, 0.0, True, 0.0, 0.0, 0.1)  # enter observe, starts blocked
    for _ in range(cycles):
        dc.update(scan, forward_m, 0.0, False, 0.0, 0.0, 0.1)  # clears - one cycle
        dc.update(scan, forward_m, 0.0, True, 5.0, 0.0, 0.1)   # blocks again


def test_door_controller_does_not_commit_before_enough_cycles():
    # Enter observe, then a single blocked->clear transition (one cycle,
    # nowhere near MIN_CYCLES_OBSERVED=3) with an enormous period - if the
    # cycle-count gate were broken this would otherwise commit easily.
    dc = SlidingDoorController()
    scan = _open_scan()
    dc.update(scan, DOOR_STANDOFF_M - 0.1, 0.0, True, 0.0, 0.0, 0.1)  # enters observe, starts blocked
    result = dc.update(scan, DOOR_STANDOFF_M - 0.1, DOOR_DIRECTION_RIGHT, False,
                        1000.0, 0.0, 0.1)
    assert dc.state != 'commit'
    assert result[2] == 'door_observe'


def test_door_controller_does_not_commit_on_left_exit():
    dc = SlidingDoorController()
    scan = _open_scan()
    _run_to_wait_window(dc, scan, DOOR_STANDOFF_M - 0.1)
    # period=5.0s clear -> plenty of predicted window, but direction is LEFT
    # (-1.0), not the committed right-exit strategy - must not commit.
    result = dc.update(scan, DOOR_STANDOFF_M - 0.1, -1.0, False, 5.0, 0.0, 0.1)
    assert dc.state != 'commit'
    assert result[2] == 'door_wait_window'


def test_door_controller_does_not_commit_when_window_too_short():
    dc = SlidingDoorController()
    scan = _open_scan()
    _run_to_wait_window(dc, scan, DOOR_STANDOFF_M - 0.1)
    # period_s=1.0 -> predicted remaining = 0.5 - 0.0, far below
    # DOOR_TRANSIT_TIME_S * DOOR_TRANSIT_SAFETY_FACTOR - must not commit.
    result = dc.update(scan, DOOR_STANDOFF_M - 0.1, DOOR_DIRECTION_RIGHT, False,
                        1.0, 0.0, 0.1)
    assert dc.state != 'commit'
    assert result[2] == 'door_wait_window'


def test_door_controller_commits_on_confirmed_right_window():
    dc = SlidingDoorController()
    scan = _open_scan()
    _run_to_wait_window(dc, scan, DOOR_STANDOFF_M - 0.1)
    period = 2.0 * DOOR_TRANSIT_TIME_S * DOOR_TRANSIT_SAFETY_FACTOR + 10.0
    result = dc.update(scan, DOOR_STANDOFF_M - 0.1, DOOR_DIRECTION_RIGHT, False,
                        period, 0.0, 0.1)
    assert dc.state == 'commit'
    linear, angular, mode = result
    assert linear == DOOR_COMMIT_LINEAR
    assert mode == 'door_commit'


def test_door_controller_commit_hugs_right_wall():
    dc = SlidingDoorController()
    scan_right_wall = _scan_with_range_at(-90.0, DOOR_RIGHT_HUG_SETPOINT_M - 0.2)
    dc.state = 'commit'
    dc._commit_elapsed_s = 0.0
    linear, angular, mode = dc.update(scan_right_wall, 10.0, DOOR_DIRECTION_RIGHT,
                                       False, 5.0, 0.0, 0.1)
    # Too close to the right wall already (closer than the hug setpoint) ->
    # steer left (positive) to open it back up, same sign convention as
    # compute_drive's own follow_right mode.
    assert angular > 0.0
    assert mode == 'door_commit'


def test_door_controller_aborts_to_wait_window_if_reblocked_mid_commit():
    dc = SlidingDoorController()
    scan = _open_scan()
    dc.state = 'commit'
    dc._commit_elapsed_s = 0.1
    result = dc.update(scan, 10.0, DOOR_DIRECTION_RIGHT, True, 5.0, 0.0, 0.1)
    assert dc.state == 'wait_window'
    assert result == (0.0, 0.0, 'door_commit_abort')


def test_door_controller_clears_after_transit_time():
    dc = SlidingDoorController()
    scan = _open_scan()
    dc.state = 'commit'
    dc._commit_elapsed_s = DOOR_TRANSIT_TIME_S - 0.05
    result = dc.update(scan, 10.0, DOOR_DIRECTION_RIGHT, False, 5.0, 0.0, 0.1)
    assert dc.state == 'cleared'
    assert result is None


def test_door_controller_cleared_returns_none():
    dc = SlidingDoorController()
    dc.state = 'cleared'
    result = dc.update(_open_scan(), 10.0, 0.0, True, 0.0, 0.0, 0.1)
    assert result is None


def test_door_controller_reset():
    dc = SlidingDoorController()
    dc.state = 'commit'
    dc._commit_elapsed_s = 1.5
    dc.reset()
    assert dc.state == 'approach'
    assert dc._commit_elapsed_s == 0.0
