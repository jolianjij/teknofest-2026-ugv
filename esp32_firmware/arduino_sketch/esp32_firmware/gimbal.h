#pragma once
#include <Arduino.h>

// Differential-mechanism pan/tilt stepper gimbal (2x NEMA17 + TMC2208 v1.2,
// STEP/DIR standalone mode). Replaces the old PCA9685-servo pan/tilt.
//
// Mixing model history (this has flipped twice - read before touching it
// again):
//   2026-08-26: confirmed on hardware as a differential -
//               tilt = motorA - motorB, pan = motorB.
//   2026-09-07: user reported motor A drives tilt only, motor B drives pan
//               only, no cross-coupling - "corrected" to motorA=tilt,
//               motorB=pan, no mixing.
//   2026-09-09: user confirmed AGAIN, in different words ("tilt moves only
//               on tilt while the other one is locked, for pan you need to
//               move both") - this is unambiguously the ORIGINAL
//               differential formula: pure tilt drives motor A alone while
//               B stays locked at 0 (tilt = A - B with B = 0 means A =
//               tilt); pure pan drives both motors together in lockstep
//               (pan = B, and A = B when tilt = 0). Reverted back to the
//               differential model as of this date.
// Current (restored) mapping: motorA = pan + tilt, motorB = pan - see
// gimbal.cpp's gimbalUpdate() for the actual formula. Given this has been
// stated three different ways across three sessions, treat the NEXT
// in-person hardware test as the tiebreaker over any further verbal
// description, per the working agreement.
//
// Control scheme: RATE control, not absolute position. A servo can be told
// "go to this angle" because it has its own internal position feedback; an
// open-loop stepper has none, so there's no way to command "go to angle X"
// without already knowing exactly where it is (which would need homing /
// limit switches - not built). Instead, stick deflection commands an
// angular VELOCITY per axis (pan/tilt, not the raw differential motors),
// ramped toward the target at GIMBAL_ACCEL_STEPS_S2 rather than snapping
// instantly, and soft-limited against GIMBAL_PAN_LIMIT_STEPS/
// GIMBAL_TILT_LIMIT_STEPS (see config.h - these are in raw steps, not
// degrees, since the gear ratio isn't measured yet).
//
// Stepping is NON-BLOCKING. gimbalUpdate() must be called every loop()
// iteration; it emits at most one step pulse per raw motor per call, timed
// against micros(), so it never blocks the safety-relevant checks (SBUS
// timeout, arm state, drivetrain mixing) also happening in loop().
//
// One axis at a time (2026-09-08): if both panStick and tiltStick are
// nonzero in the same call, only the larger-magnitude one is honored - the
// other is forced to zero for that update. This is a deliberate operating
// rule on top of the differential mixing above, not a substitute for it -
// with the mixing restored (2026-09-09) a "pure pan" command still moves
// both raw motors together and a "pure tilt" command still moves only
// motor A, exactly as expected; this rule only prevents a DIAGONAL (both
// pan and tilt nonzero at once) command from ever being formed in the
// first place, which is what makes laser-aim gain tuning tractable
// (diagonal motion makes it hard to tell which axis's gain produced what
// you're seeing) and keeps fire-control aim adjustments predictable.
// Applies uniformly to both input sources (manual stick reuse and the
// autonomous A,<pan>,<tilt>,<fire> command) since both funnel through this
// one function.

void gimbalBegin();

// Call every loop() iteration, unconditionally.
//   panStick, tiltStick: normalized direction deflection, -1..1, 0 = centered
//   panSpeedScale, tiltSpeedScale: 0..1, scales GIMBAL_MAX_STEP_RATE per
//            axis - these come from VarA/VarB acting as live speed dials,
//            NOT direction inputs.
//   enabled: drivers are only actually enabled (and stepping happens) when
//            true - callers should pass (armed && sbusOk && aim-mode-on).
void gimbalUpdate(float panStick, float tiltStick, float panSpeedScale, float tiltSpeedScale, bool enabled);

// Diagnostics only - current ramped step rate for each axis (steps/sec),
// for verifying the speed dial and acceleration ramp are actually doing
// something (see the 'G,' debug line in esp32_firmware.ino).
float gimbalGetPanRate();
float gimbalGetTiltRate();
