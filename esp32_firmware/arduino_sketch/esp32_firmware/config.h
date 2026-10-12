#pragma once

// Self-contained: this header declares sized types (uint32_t), and it is
// included before Arduino.h in some translation units, so it cannot rely on
// something else having pulled stdint in first.
#include <stdint.h>

// ---- SBUS channel mapping (FlySky, 0-indexed) ----
// Matches the 10-channel mod: CH1-4 = sticks, CH5=VarA, CH6=VarB, CH7=SwA,
// CH8=SwB, CH9=SwC, CH10=SwD.
//
// Pan/tilt reuses the drive sticks (CH_RIGHT_STICK/CH_LEFT_STICK) as direction
// inputs while SwC gates aim mode - see SBUS_CH_AIM_SWITCH below. VarA/VarB
// are NOT direction inputs; they're speed dials (see GIMBAL section below).
// This is Claude's interpretation of the requested scheme - verify it
// matches intent before trusting it.
//
// Tank mix, not arcade mix: each stick's vertical axis drives its own track
// directly (no steer+throttle combining). Channel numbers are the FlySky's
// own 1-indexed labels (CH2, CH3); SBUS channel indices below are 0-indexed,
// so CH2 -> index 1, CH3 -> index 2.
//
// Sign convention is UNVERIFIED: sbusToNormalized() returns positive when the
// raw SBUS value is above centre, which is assumed to mean "stick pushed
// forward = positive = forward motion". A prior stick (tilt) was found
// inverted on real hardware, so treat this the same way - confirm on blocks
// before trusting a forward command, per the commissioning plan's step 06.
constexpr int SBUS_CH_RIGHT_STICK = 1; // CH2, right stick vertical - right track speed+direction, pan in aim mode
constexpr int SBUS_CH_LEFT_STICK  = 2; // CH3, left stick vertical  - left track speed+direction, tilt in aim mode
constexpr int SBUS_CH_PAN_SPEED_KNOB  = 4; // CH5, VarA - scales max pan speed, 0..1
constexpr int SBUS_CH_TILT_SPEED_KNOB = 5; // CH6, VarB - scales max tilt speed, 0..1
constexpr int SBUS_CH_ARM   = 6; // CH7, SwA - 2-pos switch: disarmed vs armed (software e-stop)
constexpr int SBUS_CH_MODE  = 7; // CH8, SwB - 2-pos switch: manual vs autonomous
// SwC is a 3-POSITION switch: off = normal driving; middle = aim mode
// (drive forced to zero, CH_RIGHT_STICK/CH_LEFT_STICK become pan/tilt direction);
// full = aim mode PLUS the laser relay (PCA_CH_LASER) on. Read via the two
// thresholds below, not a simple on/off compare.
constexpr int SBUS_CH_AIM_SWITCH = 8; // CH9, SwC
// SwD moved off brake duty (2026-09-09, explicit request) - it now drives
// the tank LEDs relay directly as a manual on/off switch, replacing the old
// automatic "lit while driving" heuristic entirely.
constexpr int SBUS_CH_TANK_LEDS = 9; // CH10, SwD

// ---- Manual brake channels (2026-09-09, replaces the old single SwD brake
// switch) ----
// Per-side, on previously-unused CH1/CH4 (both ordinary spring-centered
// stick axes on this transmitter, not switches) rather than one switch for
// both sides. Deliberately asymmetric, per explicit request:
//   CH1 -> right side: thresholded on/off, mirrored to both the right relay
//          AND the right servo together (same behavior the old shared SwD
//          switch gave each side, just retriggered per-side).
//   CH4 -> left side: genuine proportional control of ONLY the left
//          mechanical brake servo (0..1 maps linearly across its
//          engaged/released angle range) - the left relay is thresholded
//          off the same channel value, not independently driven.
// Both channels are pushed in the FORWARD direction only (negative
// deflection is clamped to "no brake", not "negative brake" - see
// applyDeadzone/constrain usage in esp32_firmware.ino) since sbusToNormalized
// centers a resting stick at 0 and there is no meaningful negative brake.
constexpr int SBUS_CH_RIGHT_BRAKE      = 0; // CH1
constexpr int SBUS_CH_LEFT_BRAKE_SERVO = 3; // CH4
// Deliberately large (vs. DRIVE_STICK_DEADZONE's 0.10) per explicit request
// ("so not any slip can trigger it") - these channels sit at rest most of
// the time and a false brake engagement is more disruptive than a
// slightly-late real one. Tune down only if it feels unresponsive once
// confirmed the vehicle never brakes from stick drift at rest.
constexpr float BRAKE_CHANNEL_DEADZONE = 0.20f;

// SBUS raw channel values are ~172..1811, center ~992 (11-bit channel data)
constexpr int SBUS_MIN = 172;
constexpr int SBUS_MID = 992;
constexpr int SBUS_MAX = 1811;

// Above this value on SBUS_CH_MODE -> autonomous mode; below -> manual
constexpr int SBUS_MODE_THRESHOLD = (SBUS_MIN + SBUS_MAX) / 2;
// Above this value on SBUS_CH_ARM -> armed
constexpr int SBUS_ARM_THRESHOLD = (SBUS_MIN + SBUS_MAX) / 2;
// 3-position switch zones for SBUS_CH_AIM_SWITCH: below LOW = off, between
// LOW and HIGH = middle, above HIGH = full.
constexpr int SBUS_3POS_LOW_THRESHOLD  = SBUS_MIN + (SBUS_MAX - SBUS_MIN) / 3;
constexpr int SBUS_3POS_HIGH_THRESHOLD = SBUS_MIN + 2 * (SBUS_MAX - SBUS_MIN) / 3;

// If no valid SBUS frame arrives within this window, treat the link as lost:
// force disarmed + stop, regardless of mode.
constexpr unsigned long SBUS_TIMEOUT_MS = 500;

// Drive stick deadzone (steer/throttle) - the physical stick doesn't sit at
// EXACTLY the raw center value, so without this even a "centered" stick
// produces a tiny nonzero magnitude that still maps above the driver's ~1V
// floor (measured 1.19V at rest before this was added). 0.10 was picked to
// clear that measured ~0.079 drift with real margin - lower it if it feels
// unresponsive near center, but re-check the motor stays off at rest first.
constexpr float DRIVE_STICK_DEADZONE = 0.10f;

// ---- Drivetrain speed (analog DAC voltage, see pins.h) ----
// dacWrite() takes 0-255 representing 0-3.3V linearly.
// REVERTED 2026-09-11 (confirmed on real hardware): DAC_OFF_VALUE was
// briefly changed to a genuine 0 (0V) on the theory that "motor off" should
// be distinct from DAC_MIN_VALUE's ~1V floor - reasonable from the code
// alone, but wrong for this driver. Flashed and tested: at literal 0V the
// motor stopped responding to commands entirely ("now it dont move"). This
// driver needs the speed line held at ~1V even at rest to stay responsive -
// true 0V is not a safe/usable "off" state on this specific hardware.
// Reverted to matching DAC_MIN_VALUE, as it always was before that change.
// Do not "fix" this again without a fresh hardware test confirming it.
constexpr int DAC_OFF_VALUE = 79;                       // ~1.0V - "off" on this driver, NOT literal 0V
constexpr int DAC_MIN_VALUE = 79;                      // ~1.0V - slowest commanded speed
constexpr int DAC_MAX_VALUE = 255;                     // 3.3V - full speed

// PCA9685 PWM frequency - only the relay channels (direction/brake/laser/
// LEDs) and the gimbal's Adafruit_PWMServoDriver init still touch this;
// drivetrain speed moved off PCA9685 entirely onto the DAC pins above.
constexpr int PCA9685_PWM_FREQ_HZ = 50;

// Relay board polarity - the extremely common cheap "5V relay module"
// boards are ACTIVE LOW (a LOW input signal energizes the relay). This is
// an assumption based on that convention, NOT verified against your
// specific board - if every relay ends up backwards (on when it should be
// off), flip this.
constexpr bool RELAY_ACTIVE_LOW = true;

// Engage both brake relays (and servos) automatically whenever
// disarmed/failsafe, in addition to the manual per-side brake channels
// above - a safety default, not something explicitly requested; reconsider
// if it's not what's wanted.
constexpr bool BRAKE_ON_DISARM = true;

// How long to hold the speed line at zero after a direction reversal before
// letting speed resume. Confirmed on the real driver: it only registers a
// reversal if the speed line has zero voltage at the moment the reverse
// relay switches - if voltage is still present, the flip is ignored and the
// motor keeps running the old way. This must be long enough that the relay
// has physically settled AND the driver has sampled the new state before
// voltage returns. Tune on the vehicle: too short and reversal silently
// fails again; there is no known downside to it being generous.
constexpr uint32_t DIRECTION_REVERSAL_BRAKE_MS = 100;

// After the reversal hold above ends, don't let speed jump straight to
// whatever's commanded - ease it in linearly from DAC_MIN_VALUE (the
// slowest actual moving speed) up to the commanded value over this many ms.
// Explicit request (2026-09-11): snapping to full torque immediately after
// a direction flip was too abrupt mechanically. Only applies right after a
// reversal-triggered hold, not to normal driving. 0 disables it (old
// instant-resume behavior).
constexpr uint32_t REVERSAL_RAMP_MS = 500;

// Once a wheel starts reversing, it must keep reversing for at least this
// long before being allowed to flip back to forward - explicit request
// (2026-09-11): "you can do reverse but it needs not to be a very small
// reverse, at least one second of reverse." This is the actual fix for
// steering-driven oscillation (a noisy steering signal jittering a wheel
// between directions), replacing an earlier attempt that banned reverse
// from steering entirely and cost the vehicle its tight-pivot turning
// ability - see esp32_firmware.ino's applyReverseCommitment(). Distinct
// from DIRECTION_REVERSAL_BRAKE_MS/REVERSAL_RAMP_MS above, which handle
// the electrical side of a flip that's already been allowed to happen.
constexpr uint32_t MIN_REVERSE_COMMIT_MS = 1000;

// Reverse impact guard (2026-09-11, explicit request: "if he reverse the
// way make a safety check using the gyro values ... to not damage the
// walls of the parkours"). Backing up is blind - no rear-facing sensor
// exists - so the only way to know the vehicle has actually backed into a
// wall is from what the impact itself does to the chassis: a genuine hit
// produces an abrupt, single-loop-iteration jump in gyro magnitude that the
// vehicle's own motors cannot produce (DIRECTION_REVERSAL_BRAKE_MS holds
// zero voltage through the flip and REVERSAL_RAMP_MS eases speed in over
// half a second, so any turning the vehicle does to ITSELF while reversing
// builds up gradually, never as one sudden step). See
// checkReverseImpactGuard() in esp32_firmware.ino.
//
// UNVALIDATED PLACEHOLDER - whether this unit is rad/s or deg/s depends on
// the Adafruit_BNO055 library's default sensor configuration, which has
// never been independently confirmed on this specific board (the wire
// protocol header comment above claims rad/s; the library's own default for
// VECTOR_GYROSCOPE is commonly deg/s - this has not been resolved). Do NOT
// assume either unit when tuning this: log s_lastGyroMag on real hardware
// during normal reversing (to find a safe noise floor) and during a
// deliberate light bump test (to find a real trigger magnitude) before
// trusting this threshold at all, per the project's hardware-validation
// policy. Starting placeholder is deliberately high (unlikely to false-
// trigger on normal driving) - lower it once real numbers exist if it
// isn't sensitive enough.
constexpr float REVERSE_IMPACT_GYRO_DELTA = 80.0f;

// How long to hold the emergency electrical brake once an impact is
// detected - same order of magnitude as REVERSAL_RAMP_MS, long enough to
// actually stop residual momentum rather than just zeroing drive voltage
// for one instant.
constexpr uint32_t REVERSE_IMPACT_BRAKE_HOLD_MS = 500;

// ---- Mechanical brake servos (PCA_CH_RIGHT_BRAKE_SERVO/_LEFT_, pins.h) ----
// SEPARATE from the electrical brake relays above - these drive a physical
// brake mechanism per side. Measured 2026-09-04 with
// esp32_firmware/arduino_sketch/brake_servo_calibration/: 0 deg = engaged,
// 180 deg = fully released, same for both sides.
//
// Wired into applyMotorOutputs() as a mirror of that side's manual-brake/
// disarm condition ONLY - deliberately EXCLUDING that side's direction-
// reversal hold (removed 2026-09-09, explicit request: "remove the
// mechanical brake when reverse"). The reversal hold is ~100ms
// (DIRECTION_REVERSAL_BRAKE_MS), far shorter than a real servo's travel
// time, so engaging/releasing the servo on every reversal accomplished
// nothing but wear - the electrical relay alone is what makes reversal
// itself work (see its own comment above). Left side additionally gets
// genuine proportional control from SBUS_CH_LEFT_BRAKE_SERVO instead of a
// hard on/off snap - see the SBUS channel block above.
constexpr float BRAKE_SERVO_RIGHT_ENGAGED_DEG  = 0.0f;
constexpr float BRAKE_SERVO_RIGHT_RELEASED_DEG = 180.0f;
constexpr float BRAKE_SERVO_LEFT_ENGAGED_DEG   = 0.0f;
constexpr float BRAKE_SERVO_LEFT_RELEASED_DEG  = 180.0f;

// Generous standard hobby-servo pulse range (matches the calibration sketch).
// If a servo buzzes/strains at either engaged or released without actually
// reaching that angle, its real range is narrower than this - narrow this
// range rather than the BRAKE_SERVO_*_DEG values above.
constexpr int BRAKE_SERVO_PULSE_MIN_US = 2500;   // ~0 degrees
constexpr int BRAKE_SERVO_PULSE_MAX_US = 500;  // ~180 degrees

// ---- Aiming gimbal (differential stepper pan/tilt) ----
// TMC2208 standalone mode sets microstepping in HARDWARE via MS1/MS2 pins
// on the driver board, not here - confirm MS1=HIGH, MS2=LOW is actually
// wired for the 32 microsteps these numbers assume.
constexpr bool GIMBAL_EN_ACTIVE_LOW = true; // most TMC2208 breakouts: LOW = enabled
constexpr int GIMBAL_STEP_PULSE_US = 5;     // TMC2208 needs ~100ns min - huge margin

// Rate control, not absolute position (see gimbal.h for why). Max PAN/TILT
// step rate at full stick deflection - UNTUNED, this is a starting guess
// carried over from the face-tracking demo's working speed (500us/step ==
// 2000 steps/sec), not validated against the real differential mechanism's
// actual torque/inertia. Watch for stalling/skipped steps and lower this
// (or raise GIMBAL_STEP_PULSE_US-adjacent timing) if it does.
constexpr float GIMBAL_MAX_STEP_RATE = 2000.0f;  // steps/sec
// steps/sec^2 - ramps toward the target rate instead of snapping instantly.
// Was 4000 - at that value, reaching even a modest commanded speed took a
// well under a second, easy to mistake for "not ramping at all." Slowed
// way down so the ramp is unmistakably visible for a first test; raise it
// back up once you've confirmed it's actually working and just want it
// snappier.
constexpr float GIMBAL_ACCEL_STEPS_S2 = 600.0f;
constexpr float GIMBAL_STICK_DEADZONE = 0.05f;   // normalized, ignore stick jitter near center

// Soft position limits, in PAN/TILT STEPS from center (NOT degrees) - the
// gear/differential ratio isn't measured yet, so there's no way to express
// these as real angles. These are placeholder guesses purely to stop the
// gimbal winding into itself indefinitely during testing; recalibrate once
// the ratio is known and real angle limits can be computed. Position is a
// soft estimate only (open-loop, no encoders) - it can drift from reality
// if the mechanism ever skips steps or stalls.
constexpr long GIMBAL_PAN_LIMIT_STEPS  = 20000;
constexpr long GIMBAL_TILT_LIMIT_STEPS = 8000;

// Tilt-only drift compensation (2026-09-09, reported: pan never drifts,
// tilt drifts a bit). This is a bounded workaround, NOT a real fix - an
// open-loop stepper with no position feedback cannot detect that it has
// actually drifted, so nothing here can correct for it directly. Pan not
// drifting while tilt does is consistent with a mechanical cause specific
// to the tilt linkage (backlash, or motor A alone fighting gravity/load
// while pan drives both motors in lockstep) rather than a firmware bug -
// if there is a real fix, it is more likely a TMC2208 Vref/current
// adjustment or fixing physical play in the tilt mechanism than more code.
// Starts at 0.0 (no effect). If the drift turns out to be a small,
// roughly constant creep in ONE direction, a small constant step-rate
// bias added to motor A only when tilt is idle can approximately cancel
// it - tune this by watching the real gimbal: raise the magnitude from 0
// in small steps (try +/-20 steps/sec) until the drift stops, with the
// sign matching whichever direction it needs to counter-creep. Wrong sign
// will make the drift visibly worse immediately, not subtly - so this is
// safe to experiment with live.
constexpr float TILT_IDLE_DRIFT_COMPENSATION_STEPS_S = 0.0f;

// If no /cmd_vel message arrives within this window while in autonomous
// mode, treat it as stale and stop.
constexpr unsigned long CMD_VEL_TIMEOUT_MS = 500;

// Periodic publish rate for IMU / odom / diagnostics (milliseconds)
constexpr unsigned long PUBLISH_PERIOD_MS = 50; // 20 Hz
