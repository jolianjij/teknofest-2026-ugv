"""
ROS-independent core of mission_manager.py's drive loop: corridor-following
and stall-detection/obstacle-power-through logic, split out 2026-09-11 so it
can be unit-tested without rclpy installed (same rationale as
target_detection.py / aim_tracker.py's AcquisitionConfirmer split - see that
file's docstring). mission_manager.py imports everything from here; nothing
in this file touches ROS.

Takes a duck-typed "scan" object (anything with .ranges, .angle_increment,
.angle_min, .range_min, .range_max - a real sensor_msgs/LaserScan satisfies
this without needing to be imported here) so tests can pass a plain stand-in
object instead of constructing a real ROS message.
"""
import math

# --- Corridor-following tuning (placeholders) ---
# All of these (SAFETY_STOP_M, CAUTION_DISTANCE_M, K_LATERAL, K_HEADING,
# CRUISE_LINEAR, ...) are reasoned-about placeholders, not measured/tuned
# values - they have never been validated against the real vehicle's actual
# braking distance, turning response, or lidar mounting geometry. Do not
# trust them at anything other than a very low speed_scale (see
# serial_bridge.py's /tuning/drive) until they've been tuned on the real
# course, per the project's hardware-validation policy.
FORWARD_SECTOR_DEG = 20.0     # +/- around straight ahead, for the hard-stop check
SIDE_SECTOR_DEG = 25.0        # +/- around each side, for wall-distance sensing
SAFETY_STOP_M = 0.8           # hard stop inside this range dead ahead - PLACEHOLDER, vehicle is 1.45m long
CAUTION_DISTANCE_M = 1.8      # start slowing down inside this range dead ahead
MAX_SIDE_RANGE_M = 3.0        # beyond this, treat a side as "no wall detected", not "very far wall"
WALL_FOLLOW_SETPOINT_M = 0.8  # desired stand-off when only one wall is visible
# K_LATERAL/MAX_ANGULAR raised 2026-09-11 (explicit request: "take
# relatively large steers") - the oscillation fixes right below this
# (deadzone + slew-rate limiter) already filter out noise-sized error at
# the source, so a bigger gain on top of them amplifies genuine steering
# error into a punchier turn without reintroducing the jitter they were
# built to remove. If turns overshoot/oscillate at the real speed_scale,
# lower K_LATERAL back down before touching the deadzones again.
K_LATERAL = 1.0               # steering gain from wall-distance error
K_HEADING = 0.3               # steering gain from zeroed-heading drift correction
MAX_ANGULAR = 1.0             # clamp, normalized -1..1 (further scaled by serial_bridge's /tuning/drive)
# Raised 0.5 -> 0.7 (2026-09-11, explicit report: "the motor is running
# slowly"). 0.5 was itself an explicit earlier choice ("I want the speed
# normal speed to be the half of the total") - this supersedes it, same
# pattern as this file's other same-day gain revisions (K_LATERAL, etc.):
# implement the correction, document the trade-off, adjust if the real
# vehicle overshoots. Real trade-off worth knowing: linear is clamped to
# 1.0 AFTER climb_boost multiplies it (see compute_drive's return line), so
# a higher baseline leaves less multiplicative headroom for the climb
# boost to actually do anything before saturating - at 0.7, climb_boost
# saturates the clamp at ~1.43x (a ~43% max boost) instead of the full
# CLIMB_BOOST_MAX=1.0 (+100%) headroom 0.5 left. If climbs start stalling
# for lack of torque, raise CLIMB_BOOST_MAX rather than lowering this back
# down - the two now genuinely compete for the same 0..1 ceiling.
CRUISE_LINEAR = 0.7           # normalized -1..1 cruise speed when unobstructed

# Oscillation margin (2026-09-11, explicit hardware finding: steering noise
# was jittering the commanded direction across zero on one wheel, and on
# this driver crossing into reverse costs real torque/momentum - see
# motor_mixer.cpp's mixDifferential for the ESP32-side half of this fix).
# WALL_ERROR_DEADZONE_M ignores wall-distance differences/offsets smaller
# than this - normal lidar range noise, not a real drift needing correction.
# HEADING_DEADZONE_RAD does the same for yaw drift. Neither number is
# measured; both are a reasoned starting margin - if the vehicle still
# hunts/oscillates, widen these before raising K_LATERAL/K_HEADING to
# compensate (a bigger gain makes noise-driven jitter worse, not better).
WALL_ERROR_DEADZONE_M = 0.05
HEADING_DEADZONE_RAD = 0.03

# Corner-turn boost (2026-09-11, explicit request: "I want something
# guaranteed for the turn") - see the full reasoning in compute_drive()
# where it's applied. 1.0 means no boost far from any wall; up to this much
# EXTRA multiplier on steering strength as the vehicle closes in on
# SAFETY_STOP_M dead ahead, so it actively commits to a turn instead of
# just slowing down with a possibly-too-weak lateral correction. Not yet
# hardware-tuned - if turns still don't clear a corner in time, raise this;
# if it swings too hard right before a hard_stop, lower it.
CORNER_TURN_BOOST_MAX = 1.5

# Side-wall contact guard (2026-09-11, explicit request: "make sure that the
# robot can pass crossing level [curved ways] and sliding door without
# touch the walls"). Everything above only reacts to a wall dead AHEAD
# (SAFETY_STOP_M/CAUTION_DISTANCE_M) - through a curve (the two U-turns
# especially, a real geometry squeeze: robot_project_plan.md flags the
# spec's 5m turning-radius requirement against a 3m corridor as worth
# checking once chassis dimensions were measured) the vehicle's FLANK can
# close on a wall while the forward sector still reads clear. This adds an
# independent check on the two SIDE sectors already being read for
# corridor-centering below, so a curve that's tightening faster than the
# centering response can correct for it gets a hard override instead of a
# graceful-but-too-slow steer.
#
# SIDE_CONTACT_MARGIN_M is a RAW LIDAR RANGE threshold, not a hull-edge
# clearance - the lidar sits near the vehicle's centerline, so a side ray's
# range is approximately the distance from that centerline to the wall, and
# the hull extends TRACK_SEPARATION_M/2 = 0.55m (real measurement, see
# CLAUDE.md/URDF) from there. 0.65m leaves only ~0.10m of hull clearance
# once triggered - deliberately tight (a bigger margin would trip on normal
# WALL_FOLLOW_SETPOINT_M=0.8m following and fight the centering logic
# constantly) but still meant to be reached rarely, only when centering has
# genuinely failed to keep up. Not measured against the real lidar mount
# offset - tighten if the hull is ever observed touching before this fires.
SIDE_CONTACT_MARGIN_M = 0.65
# Escape steer strength once triggered - strong enough to matter, not
# MAX_ANGULAR so there is still some headroom if both sides are critical at
# once (see compute_drive - that case holds heading instead of picking a
# side to avoid steering harder into whichever wall turns out to be closer).
SIDE_CONTACT_STEER = 0.6

# Climb power boost (2026-09-11, explicit request: "make sure that the
# motor values of the vehicle change based on gyro values to make it able
# [to] pass all parkour stages"). As the vehicle's pitch increases - nose
# tilting up while climbing a ramp/step - linear power is boosted
# proportionally, so it has the torque to get over the obstacle instead of
# stalling partway up (this is a genuinely different signal from
# StallDetector below, which only reacts AFTER forward acceleration has
# already stopped - this boosts proactively, from the tilt itself, before
# that happens). PITCH_CLIMB_SIGN is UNCONFIRMED: the IMU's physical
# mounting on this vehicle has only ever been validated for YAW (via
# heading_reference.py) - nothing has confirmed which sign of pitch means
# "nose up" on this specific mount. Tilt the vehicle nose-up by hand and
# check quat_to_pitch_deg()'s output is positive before trusting this; if
# it reads negative while climbing, flip this one constant rather than the
# boost math below it.
PITCH_CLIMB_SIGN = 1.0
CLIMB_PITCH_FULL_BOOST_DEG = 15.0  # pitch angle at which the boost reaches its max - not measured, a reasoned guess
CLIMB_BOOST_MAX = 1.0              # +100% linear power at CLIMB_PITCH_FULL_BOOST_DEG and beyond

# The lidar's own angle_min=0 reference does not line up with the vehicle's
# actual forward direction (reported 2026-09-09 via the dashboard's lidar
# plot rendering front at the back - see web_dashboard.py's drawLidar for
# the matching constant). This is a mounting/frame issue in the raw /scan
# data itself, not specific to any one consumer, so every angle this file
# computes against the vehicle's actual forward/left/right must correct for
# it. Best guess is a clean 180 degrees; if that is not exactly right,
# change this constant (and the matching one in web_dashboard.py) rather
# than adjusting individual sector angles below.
LIDAR_FRONT_OFFSET_RAD = math.pi


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def quat_to_pitch_deg(w: float, x: float, y: float, z: float) -> float:
    """Pitch (degrees) from an orientation quaternion - the same standard
    aerospace ZYX extraction heading_reference.py's quat_to_euler() uses for
    yaw, isolated here since compute_drive only needs pitch. Sign is
    whatever the IMU's physical mounting gives - see PITCH_CLIMB_SIGN."""
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # guard asin() domain against float noise at the poles
    return math.degrees(math.asin(sinp))


def _deadzone(value, threshold):
    """Zeroes out anything within threshold of zero, otherwise removes the
    threshold itself from the magnitude so the response starts at zero right
    at the deadzone edge instead of jumping - same reasoning as the ESP32
    firmware's applyDeadzone() for stick input."""
    if abs(value) <= threshold:
        return 0.0
    return value - threshold if value > 0 else value + threshold


def min_range_in_sector(scan, center_deg: float, half_width_deg: float) -> float:
    """Smallest valid range within +/- half_width_deg of center_deg (0 = straight
    ahead, +90 = left, -90 = right, matching REP103's CCW-positive convention).
    Returns inf if nothing valid was found in that sector (open space, or the
    sector isn't within this lidar's actual field of view)."""
    if not scan.ranges or scan.angle_increment == 0.0:
        return float('inf')
    lo = math.radians(center_deg - half_width_deg)
    hi = math.radians(center_deg + half_width_deg)
    best = float('inf')
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment + LIDAR_FRONT_OFFSET_RAD
        # Wrap into (-pi, pi] - the +offset above can push angle past +/-pi,
        # which would make it silently miss whichever sector straddles that
        # wrap boundary (not currently forward/left/right, but don't rely on
        # that staying true if sector angles change later).
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if angle < lo or angle > hi:
            continue
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        if r < best:
            best = r
    return best


def compute_drive(scan, yaw_rel: float, pitch_deg: float = 0.0):
    """Returns (linear, angular, mode_str). Pure function, no ROS/node state,
    so it can be unit-tested without spinning a node. pitch_deg defaults to
    0.0 (no climb boost) so existing callers/tests that don't pass it are
    unaffected."""
    forward = min_range_in_sector(scan, 0.0, FORWARD_SECTOR_DEG)
    if forward < SAFETY_STOP_M:
        return 0.0, 0.0, 'hard_stop'

    if forward < CAUTION_DISTANCE_M:
        speed_scale = clamp(
            (forward - SAFETY_STOP_M) / (CAUTION_DISTANCE_M - SAFETY_STOP_M), 0.0, 1.0
        )
    else:
        speed_scale = 1.0

    left = min_range_in_sector(scan, 90.0, SIDE_SECTOR_DEG)
    right = min_range_in_sector(scan, -90.0, SIDE_SECTOR_DEG)
    have_left = left < MAX_SIDE_RANGE_M
    have_right = right < MAX_SIDE_RANGE_M

    # Side-contact guard - see SIDE_CONTACT_MARGIN_M's own comment. Checked
    # before the normal centering math below (which is a proportional
    # response, too gentle to trust once a wall is THIS close) and returned
    # immediately: stop advancing, steer away from whichever side is
    # critical. Pinched on both sides at once holds heading instead of
    # picking one - steering either way risks closing faster on the other
    # wall, and both-critical is already a situation compute_drive alone
    # cannot resolve safely.
    left_critical = have_left and left < SIDE_CONTACT_MARGIN_M
    right_critical = have_right and right < SIDE_CONTACT_MARGIN_M
    if left_critical or right_critical:
        if left_critical and right_critical:
            escape_angular = 0.0
        elif left_critical:
            escape_angular = -SIDE_CONTACT_STEER  # steer right, away from the left wall
        else:
            escape_angular = SIDE_CONTACT_STEER   # steer left, away from the right wall
        return 0.0, escape_angular, 'side_contact_risk'

    if have_left and have_right:
        # Steer toward whichever side has more room. left>right -> turn left
        # (positive, CCW) away from the closer right-hand wall, and vice versa.
        angular_lateral = K_LATERAL * _deadzone(left - right, WALL_ERROR_DEADZONE_M)
        mode = 'center'
    elif have_left:
        # Only the left wall is visible - hold WALL_FOLLOW_SETPOINT_M off it.
        # Too close (left < setpoint) -> turn right (negative) to move away.
        angular_lateral = K_LATERAL * _deadzone(left - WALL_FOLLOW_SETPOINT_M, WALL_ERROR_DEADZONE_M)
        mode = 'follow_left'
    elif have_right:
        # Too close (right < setpoint) -> turn left (positive) to move away.
        angular_lateral = K_LATERAL * _deadzone(WALL_FOLLOW_SETPOINT_M - right, WALL_ERROR_DEADZONE_M)
        mode = 'follow_right'
    else:
        angular_lateral = 0.0
        mode = 'open'

    # Corner-turn boost (2026-09-11, explicit request: "I want something
    # guaranteed for the turn"). Without this, steering strength only comes
    # from the raw left/right distance difference - if that difference is
    # still small while the vehicle is closing in on a wall dead ahead (a
    # square/narrow corner, not a gradual bend), the turn it commits to can
    # be too weak to actually clear the corner before hitting
    # CAUTION_DISTANCE_M's speed cutoff, risking a hard_stop nose-first into
    # the wall instead of a completed turn. speed_scale already tracks
    # exactly this "how close to the wall ahead" measure (1.0 far away, 0.0
    # at the SAFETY_STOP_M edge) - reusing it here means steering strength
    # ramps up automatically as the corner gets closer, up to
    # CORNER_TURN_BOOST_MAX extra multiplier right at the wall, instead of
    # only ever scaling down with distance the way linear speed does. Safe
    # when there is no real opening either (both sides roughly equal or both
    # missing): angular_lateral is already near zero in that case, and
    # boosting a near-zero value stays near zero - this only sharpens a turn
    # that was already being attempted, it does not invent one.
    corner_boost = 1.0 + (1.0 - speed_scale) * CORNER_TURN_BOOST_MAX
    angular_lateral *= corner_boost

    # yaw_rel > 0 means the vehicle has rotated left of the heading it had
    # when autonomous engaged - correct back toward 0 by turning right.
    angular_heading = -K_HEADING * _deadzone(yaw_rel, HEADING_DEADZONE_RAD)
    angular = clamp(angular_lateral + angular_heading, -MAX_ANGULAR, MAX_ANGULAR)

    # Climb boost - see the CLIMB_* constants' own comment for the full
    # reasoning. Only ever boosts (never penalizes descending, hence the
    # max(0.0, ...) - going downhill needs no extra torque), and is capped
    # at CLIMB_BOOST_MAX extra multiplier once pitch reaches
    # CLIMB_PITCH_FULL_BOOST_DEG.
    climb_deg = max(0.0, PITCH_CLIMB_SIGN * pitch_deg)
    climb_boost = 1.0 + CLIMB_BOOST_MAX * clamp(climb_deg / CLIMB_PITCH_FULL_BOOST_DEG, 0.0, 1.0)
    linear = clamp(CRUISE_LINEAR * speed_scale * climb_boost, 0.0, 1.0)

    return linear, angular, mode


# --- Turn-then-advance strategy (2026-09-11, explicit request) ---
# "in the situation of turns use the strategy changing angular then moving
# forward a bit changing angular then moving forward a bit with the gyro
# as a reference. Over all improve our robot as a maze solver robot."
#
# compute_drive() above still computes ONE continuous reactive signal each
# tick - how much turning the current wall/heading error calls for right
# now (its `angular` return value). What changes here is how that signal
# gets USED: instead of always blending it straight into ongoing forward
# motion, TurnAdvanceController below turns it into two committed phases,
# the way a maze-solving robot actually behaves:
#   ALIGN   - the error is large enough to call it a real turn, not a
#             gentle correction. Forward motion stops (the caller forces
#             linear=0), and the vehicle rotates in place, closed-loop
#             against the same live error signal (which is itself already
#             gyro-referenced via angular_heading's yaw_rel term - see
#             compute_drive above) until the error shrinks back down.
#   ADVANCE - error is small. Drive forward at normal speed with only a
#             small trim correction to hold the heading steady - not a
#             full turn.
# This directly satisfies "decrease speed or even stop for a strong turn"
# too, for free: ALIGN's linear=0 already IS that stop, triggered by
# exactly the same signal that would otherwise demand a strong turn.
#
# It also composes cleanly with the no-reverse-in-autonomous rule
# (motor_mixer.cpp): with linear=0 during ALIGN, mixDifferential's
# [0, ...] clamp turns a nonzero angular into a genuine pivot - one wheel
# held at 0, the other driving - automatically. "Rotate about yourself" is
# satisfied by that pivot, not by a true opposite-direction spin, staying
# consistent with the explicit "robot going backward" correction from
# earlier the same day.
#
# ENTER/EXIT are two different thresholds (not one) so the state can't
# chatter right at the boundary - the same hysteresis-margin reasoning as
# gimbal.cpp's DOMINANCE_HYSTERESIS for axis-switching. All values below
# are compared against compute_drive()'s `angular` return value directly,
# which is already a NORMALIZED -1..1 command (clamp(..., -MAX_ANGULAR,
# MAX_ANGULAR), MAX_ANGULAR=1.0) - NOT a raw radian heading error, despite
# being built from angle-shaped terms internally. TURN_ALIGN_ENTER must
# stay above ADVANCE_ANGULAR_TRIM_MAX or the trim clamp below can never
# actually apply - any signal big enough to reach it would already have
# tripped ALIGN first. All four numbers are reasoned starting points, not
# measured - watch for it aligning too eagerly on minor error (raise
# TURN_ALIGN_ENTER) or not committing hard enough to actually clear a real
# corner (raise ALIGN_ANGULAR_MAX) when tuning on the real vehicle.
TURN_ALIGN_ENTER = 0.4        # |angular| above this while ADVANCING -> switch to ALIGN
TURN_ALIGN_EXIT = 0.15        # |angular| below this while ALIGNING -> switch back to ADVANCE
ALIGN_ANGULAR_MAX = 0.7       # rotation strength while aligning, normalized -1..1
ADVANCE_ANGULAR_TRIM_MAX = 0.15  # small heading-hold correction allowed while advancing - not a full turn


class TurnAdvanceController:
    """Pure, unit-testable - no ROS. Feed it compute_drive()'s raw angular
    value each tick; tracks which phase the vehicle is in and returns the
    angular command actually appropriate to that phase. Does not touch
    linear - the caller reads .aligning and forces linear to 0 itself
    while True (see mission_manager.py's drive_loop)."""

    def __init__(self):
        self.aligning = False

    def update(self, angular_target: float) -> float:
        mag = abs(angular_target)
        if self.aligning:
            if mag < TURN_ALIGN_EXIT:
                self.aligning = False
        else:
            if mag > TURN_ALIGN_ENTER:
                self.aligning = True

        if self.aligning:
            return clamp(angular_target, -ALIGN_ANGULAR_MAX, ALIGN_ANGULAR_MAX)
        return clamp(angular_target, -ADVANCE_ANGULAR_TRIM_MAX, ADVANCE_ANGULAR_TRIM_MAX)

    def reset(self):
        self.aligning = False


# Slew-rate limit on the published angular command (2026-09-11) - a second,
# independent layer against the same oscillation the deadzones above and
# motor_mixer.cpp's reverse-clamp target: even a REAL (non-noise) change in
# wall distance can otherwise ask for a big angular swing in one tick,
# which is exactly what pushes a wheel's mix across zero into reverse.
# Capping how fast the commanded angular value itself can change smooths
# that transition out before it ever reaches the ESP32. Raised alongside
# K_LATERAL/MAX_ANGULAR above (explicit request: "take relatively large
# steers") - goes from 0 to the new MAX_ANGULAR=1.0 in ~0.33s, a bit
# quicker to commit than the original 0.4s-to-0.8 pace, while still eased
# rather than snapped. If steering still feels twitchy, lower it; if it
# feels sluggish to react to a real wall, raise it.
MAX_ANGULAR_SLEW_PER_S = 3.0


class SteeringRateLimiter:
    """Pure, unit-testable - no ROS. Feed it (target_angular, dt) every
    drive-loop tick; returns the rate-limited angular command actually safe
    to publish this tick."""

    def __init__(self):
        self.value = 0.0

    def update(self, target_angular: float, dt: float) -> float:
        max_step = MAX_ANGULAR_SLEW_PER_S * dt
        delta = clamp(target_angular - self.value, -max_step, max_step)
        self.value += delta
        return self.value

    def reset(self):
        self.value = 0.0


# --- Stall detection / obstacle power-through (2026-09-11, explicit request:
# "if it stalls in an obstacle it needs to increase speed so it could pass
# vertical obstacle bumpers"). PLACEHOLDER VALUES, UNVALIDATED - same
# hardware-validation caveat as the corridor gains above.
#
# There are no wheel encoders (see Open Decisions in CLAUDE.md), so "stalled"
# can't be measured directly - it's inferred from the IMU already on the
# ESP32 (forwarded via /esp32/imu): if the drive loop has commanded
# meaningful forward motion for STALL_DETECT_S continuously while the IMU's
# forward-axis acceleration stays under STALL_ACCEL_THRESHOLD_MPS2 (not
# cruising at constant speed - genuinely not gaining velocity), treat it as
# stuck against something and boost power for a bounded window to try to
# power through.
#
# Bounded and self-limiting ON PURPOSE: this is a blind boost with no way to
# confirm afterward that it actually worked (still no encoders), so it must
# not become "push harder forever against something it can't climb" - that
# risks motor/gearbox damage or just grinding the tracks. After
# STALL_BOOST_MAX_S of boosted power, if still stalled, give up and
# hard-stop instead of continuing to escalate, then wait STALL_COOLDOWN_S
# before trying again.
#
# IMU_FORWARD_AXIS assumes the accelerometer's +X axis is mounted aligned
# with the vehicle's forward direction - not measured, same
# placeholder-until-physically-confirmed status as LIDAR_FRONT_OFFSET_RAD
# above. If forward acceleration reads on a different axis/sign on the real
# unit, fix the axis/sign here rather than the thresholds.
STALL_MIN_COMMAND = 0.25          # only consider a stall while genuinely trying to move, not idling
STALL_ACCEL_THRESHOLD_MPS2 = 0.4  # below this forward accel, treat as "not actually accelerating"
STALL_DETECT_S = 1.5              # how long that has to hold before calling it a stall
STALL_BOOST_LINEAR = 0.85         # normalized -1..1 - boosted forward command while powering through
STALL_BOOST_MAX_S = 2.5           # give up (hard-stop) if still stalled after this long at boost
STALL_COOLDOWN_S = 4.0            # after giving up, don't re-attempt a boost for this long
IMU_STALE_S = 0.5                 # /esp32/imu publishes at ~18-20Hz - no accel signal, no stall detection


class StallDetector:
    """Pure, unit-testable state machine - no ROS. Feed it
    (commanded_linear, forward_accel_mps2, dt) every drive-loop tick; it
    tracks whether the vehicle looks stuck trying to move and whether a
    power-through boost should currently be applied, enforcing the
    bounded-attempt safety rule above (see module comment)."""

    def __init__(self):
        self._low_accel_s = 0.0
        self._boosting_s = 0.0
        self._cooldown_s = 0.0
        self.boosting = False
        self.gave_up = False

    @property
    def in_cooldown(self) -> bool:
        """True only while an actual cooldown is counting down after a give-
        up - unlike gave_up (which stays True indefinitely as a one-shot
        event flag until the next stall attempt), this clears itself once
        the cooldown timer actually expires. Use this for status/dashboard
        reporting; use gave_up for one-shot logging of the give-up event
        itself."""
        return self._cooldown_s > 0.0

    def update(self, commanded_linear: float, forward_accel_mps2: float, dt: float) -> bool:
        """Returns True while a power-through boost should be applied."""
        if self._cooldown_s > 0.0:
            self._cooldown_s = max(0.0, self._cooldown_s - dt)

        trying_to_move = abs(commanded_linear) > STALL_MIN_COMMAND
        low_accel = abs(forward_accel_mps2) < STALL_ACCEL_THRESHOLD_MPS2

        if self.boosting:
            self._boosting_s += dt
            if not low_accel or not trying_to_move:
                # made it through (or the command dropped away) - stop boosting
                self.boosting = False
                self._boosting_s = 0.0
                self._low_accel_s = 0.0
            elif self._boosting_s >= STALL_BOOST_MAX_S:
                # bounded-attempt safety rule: don't escalate forever
                self.boosting = False
                self._boosting_s = 0.0
                self._low_accel_s = 0.0
                self.gave_up = True
                self._cooldown_s = STALL_COOLDOWN_S
            return self.boosting

        if not trying_to_move:
            self._low_accel_s = 0.0
            return False

        if self._cooldown_s > 0.0:
            return False

        self._low_accel_s = self._low_accel_s + dt if low_accel else 0.0
        if self._low_accel_s >= STALL_DETECT_S:
            self.boosting = True
            self._boosting_s = 0.0
            self.gave_up = False
            self._low_accel_s = 0.0
        return self.boosting


# --- Sliding door commit logic (Stage 6, behavior string "sliding_doors" -
# 2026-09-11, explicit request: "make sure that the robot can pass ...
# sliding door without touch the walls"). Implements the strategy already
# laid out in docs/run2-autonomy-plan.html's "Sliding door" card: stop
# outside the swept volume, watch several full cycles to learn the door's
# period and confirm it clears to the RIGHT (that doc's committed strategy -
# a left exit or an ambiguous read leaves too little margin and risks a
# strike), then commit only into a predicted clear window comfortably
# longer than the time actually needed to cross, hugging the right wall on
# the way through. Abort-back-to-observing is a first-class transition, not
# an error path - and per that doc's failure-doctrine table, THIS stage's
# own timeout fallback is explicitly "hold and keep re-observing", unlike
# most other stages ("keep driving straight") - creeping blindly into a
# moving door on a timeout is exactly the collision this exists to prevent,
# so update() never invents forward motion on its own.
#
# Fed by /door/state (door_direction_node.py's DoorTracker output) plus the
# same lidar `scan` compute_drive() already reads - no new sensor wiring
# needed. Pure, unit-testable, no ROS - mission_manager.py owns feeding it
# scan/door state each tick and switching between this and the baseline
# compute_drive() based on the active stage's behavior string.
DOOR_STANDOFF_M = 1.8              # stop this far out, outside the door's swept volume - reasoned guess, not measured
MIN_CYCLES_OBSERVED = 3            # per the doc: watch at least this many full cycles before trusting period/direction
DOOR_TRANSIT_TIME_S = 3.0          # time needed to cross at DOOR_COMMIT_LINEAR - PLACEHOLDER, depends on real DAC-to-m/s speed calibration (not yet measured, see CLAUDE.md) - recalibrate once that exists, and again on-site once the real door period is measured (5-minute self-trial window)
DOOR_TRANSIT_SAFETY_FACTOR = 1.5   # commit only if the predicted clear window exceeds transit time by this margin
DOOR_COMMIT_LINEAR = 0.85          # confident push through the window - faster than CRUISE_LINEAR (raised alongside it 2026-09-11 to stay meaningfully quicker), since hesitation burns the window
DOOR_RIGHT_HUG_SETPOINT_M = 0.5    # tighter standoff off the right wall specifically while committing (vs. WALL_FOLLOW_SETPOINT_M's 0.8) - biases away from the door's swept path
DOOR_OBSERVE_TIMEOUT_S = 20.0      # re-arms the observe timer rather than acting on it - see module comment on this stage's own failure doctrine
DOOR_DIRECTION_RIGHT = 1.0         # matches door_tracking.DIRECTION_RIGHT - duplicated as a literal rather than imported so this file keeps zero deps beyond math/stdlib


class SlidingDoorController:
    """Pure, unit-testable - no ROS. States:
      'approach'    - hand back to the baseline compute_drive() (update()
                       returns None) until the standoff range is reached.
      'observe'     - stopped, counting full blocked->clear cycles.
      'wait_window' - stopped, enough cycles seen; watching for a
                       confirmed-clear, right-exited window big enough to
                       commit into.
      'commit'      - timed drive through, hugging the right wall.
      'cleared'     - done; update() returns None (caller falls back to the
                       baseline) until reset() starts a fresh attempt.
    update() returns None whenever the caller should use compute_drive()'s
    own output instead (approach, cleared); otherwise (linear, angular,
    mode)."""

    def __init__(self):
        self.state = 'approach'
        self._cycles_observed = 0
        self._was_blocked = False
        self._observe_elapsed_s = 0.0
        self._commit_elapsed_s = 0.0

    def reset(self):
        self.__init__()

    def update(self, scan, forward_m: float, door_direction: float, door_blocked: bool,
               door_period_s: float, door_clear_elapsed_s: float, dt: float):
        if self.state == 'approach':
            if forward_m < DOOR_STANDOFF_M:
                self.state = 'observe'
                self._cycles_observed = 0
                self._was_blocked = door_blocked
                self._observe_elapsed_s = 0.0
                return 0.0, 0.0, 'door_observe'
            return None

        if self.state == 'cleared':
            return None

        if self.state == 'commit':
            if door_blocked:
                # Abort-and-re-wait - first-class, not an error path. Already
                # proved the period/direction this attempt; no need to
                # re-count cycles, just re-time the next window.
                self.state = 'wait_window'
                self._cycles_observed = MIN_CYCLES_OBSERVED
                self._observe_elapsed_s = 0.0
                self._was_blocked = door_blocked
                return 0.0, 0.0, 'door_commit_abort'
            self._commit_elapsed_s += dt
            if self._commit_elapsed_s >= DOOR_TRANSIT_TIME_S:
                self.state = 'cleared'
                return None
            return self._commit_step(scan)

        # 'observe' or 'wait_window' - a cycle completes each time the gate
        # transitions blocked -> clear.
        if self._was_blocked and not door_blocked:
            self._cycles_observed += 1
        self._was_blocked = door_blocked

        ready = self._cycles_observed >= MIN_CYCLES_OBSERVED and door_period_s > 0.0
        self.state = 'wait_window' if ready else 'observe'

        if ready and not door_blocked and door_direction == DOOR_DIRECTION_RIGHT:
            # period_s is a FULL cycle (clear-to-clear); half of it is a
            # simplifying estimate of how long the clear half typically
            # lasts, not a measured split - reasoned starting point, same
            # status as every other untuned constant here, refine on-site
            # once the real door's clear/blocked split is observed.
            predicted_remaining_s = max(0.0, (door_period_s / 2.0) - door_clear_elapsed_s)
            if predicted_remaining_s > DOOR_TRANSIT_TIME_S * DOOR_TRANSIT_SAFETY_FACTOR:
                self.state = 'commit'
                self._commit_elapsed_s = 0.0
                return self._commit_step(scan)

        self._observe_elapsed_s += dt
        if self._observe_elapsed_s >= DOOR_OBSERVE_TIMEOUT_S:
            self._observe_elapsed_s = 0.0
        return 0.0, 0.0, f'door_{self.state}'

    @staticmethod
    def _commit_step(scan):
        right = min_range_in_sector(scan, -90.0, SIDE_SECTOR_DEG)
        if right < MAX_SIDE_RANGE_M:
            angular = K_LATERAL * _deadzone(DOOR_RIGHT_HUG_SETPOINT_M - right, WALL_ERROR_DEADZONE_M)
        else:
            angular = 0.0
        angular = clamp(angular, -MAX_ANGULAR, MAX_ANGULAR)
        return DOOR_COMMIT_LINEAR, angular, 'door_commit'
