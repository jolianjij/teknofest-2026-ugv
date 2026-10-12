#include "gimbal.h"
#include "pins.h"
#include "config.h"
#include <math.h>

// Per-motor non-blocking step generator state. Differential mechanism -
// see gimbal.h for the mixing formula and its history (confirmed, then
// "corrected" away, then confirmed again - as of 2026-09-09 it IS a
// differential).
struct StepperAxis {
    int dirPin;
    int stepPin;
    unsigned long periodUs = 0; // 0 = not stepping right now
    unsigned long lastStepUs = 0;
    bool dirPositive = true;
};

static StepperAxis motorA{PIN_GIMBAL_PAN_DIR, PIN_GIMBAL_PAN_STEP};   // raw differential motor "A" (naming kept from the pins, see pins.h)
static StepperAxis motorB{PIN_GIMBAL_TILT_DIR, PIN_GIMBAL_TILT_STEP}; // raw differential motor "B" (naming kept from the pins, see pins.h)
static bool driversEnabled = false;

// Pan/tilt logical state - acceleration ramping and soft limits operate
// here, not on the raw motors, since "limit" and "speed" are concepts that
// belong to the pan/tilt axes a human actually controls.
static float panCurrentRate = 0.0f;  // steps/sec, ramped
static float tiltCurrentRate = 0.0f;
static long panPositionSteps = 0;    // soft estimate, open-loop, no encoders
static long tiltPositionSteps = 0;
static unsigned long lastUpdateUs = 0;

// One-axis-at-a-time hysteresis state (see gimbalUpdate) - persists across
// calls so a near-tie between panStick/tiltStick doesn't flip the winning
// axis every loop iteration.
static bool s_tiltIsDominant = false;

static void setAxisTarget(StepperAxis &axis, float stepsPerSec) {
    if (fabsf(stepsPerSec) < 1.0f) {
        axis.periodUs = 0;
        return;
    }
    bool wantPositive = stepsPerSec > 0;
    if (wantPositive != axis.dirPositive) {
        axis.dirPositive = wantPositive;
        digitalWrite(axis.dirPin, wantPositive ? HIGH : LOW);
    }
    axis.periodUs = (unsigned long)(1000000.0f / fabsf(stepsPerSec));
}

static void serviceAxis(StepperAxis &axis, unsigned long nowUs) {
    if (axis.periodUs == 0) return;
    if (nowUs - axis.lastStepUs < axis.periodUs) return;
    digitalWrite(axis.stepPin, HIGH);
    delayMicroseconds(GIMBAL_STEP_PULSE_US);
    digitalWrite(axis.stepPin, LOW);
    axis.lastStepUs = nowUs;
}

static void setDriversEnabled(bool enable) {
    if (enable == driversEnabled) return;
    driversEnabled = enable;
    bool activeLevel = GIMBAL_EN_ACTIVE_LOW ? LOW : HIGH;
    bool idleLevel = GIMBAL_EN_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(PIN_GIMBAL_EN, enable ? activeLevel : idleLevel);
}

// Ramps `current` toward `target` at `accel` (units/sec^2) over `dt` seconds.
static float rampToward(float current, float target, float accel, float dt) {
    float maxDelta = accel * dt;
    float delta = target - current;
    if (delta > maxDelta) delta = maxDelta;
    if (delta < -maxDelta) delta = -maxDelta;
    return current + delta;
}

// If already at/past a limit and still pushing further that way, zero the
// target so rampToward decelerates back to a stop instead of holding
// against the limit indefinitely.
static float clampTargetAtLimit(float target, long position, long limit) {
    if (position >= limit && target > 0) return 0.0f;
    if (position <= -limit && target < 0) return 0.0f;
    return target;
}

void gimbalBegin() {
    pinMode(PIN_GIMBAL_EN, OUTPUT);
    pinMode(PIN_GIMBAL_PAN_DIR, OUTPUT);
    pinMode(PIN_GIMBAL_PAN_STEP, OUTPUT);
    pinMode(PIN_GIMBAL_TILT_DIR, OUTPUT);
    pinMode(PIN_GIMBAL_TILT_STEP, OUTPUT);
    // pinMode(OUTPUT) leaves the pin LOW, but StepperAxis::dirPositive
    // defaults to true (i.e. "the DIR pin is currently HIGH") - without this,
    // an axis whose FIRST commanded direction is positive never gets its DIR
    // pin written at all (setAxisTarget only writes on a *change* from the
    // tracked state), so it silently steps in the physical direction
    // matching LOW while the firmware believes it already set HIGH. Explicit
    // writes here keep the tracked state and the real pin level in agreement
    // from the very first step.
    digitalWrite(motorA.dirPin, HIGH);
    digitalWrite(motorB.dirPin, HIGH);
    setDriversEnabled(false); // start disabled until armed, same convention as the drive motors
    lastUpdateUs = micros();
}

void gimbalUpdate(float panStick, float tiltStick, float panSpeedScale, float tiltSpeedScale, bool enabled) {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; // guard against the first call / a wrapped/huge gap

    setDriversEnabled(enabled);

    if (!enabled) {
        // Decelerate to a stop rather than snapping to zero, then hold.
        panCurrentRate = rampToward(panCurrentRate, 0.0f, GIMBAL_ACCEL_STEPS_S2, dt);
        tiltCurrentRate = rampToward(tiltCurrentRate, 0.0f, GIMBAL_ACCEL_STEPS_S2, dt);
    } else {
        if (fabsf(panStick) < GIMBAL_STICK_DEADZONE) panStick = 0;
        if (fabsf(tiltStick) < GIMBAL_STICK_DEADZONE) tiltStick = 0;

        // Only one axis may move at a time - a deliberate operating rule
        // (2026-09-08), not a mechanism limitation: simultaneous diagonal
        // motion makes laser-aim tuning harder to reason about (you can't
        // tell which axis's gain is causing what you're seeing) and makes
        // fine aim adjustments during actual shooting less predictable.
        // The currently-dominant axis keeps control until the OTHER axis's
        // deflection exceeds it by DOMINANCE_HYSTERESIS - a plain "bigger
        // wins" comparison would flip the winner every single loop whenever
        // the two are near a tie (a diagonal stick position, or noisy
        // vision-tracking error), producing jittery motion instead of the
        // clean single-axis behavior this rule exists for. The loser is
        // forced to zero, which rampToward() then decelerates smoothly on
        // its own normal curve - not a hard snap.
        if (panStick != 0.0f && tiltStick != 0.0f) {
            const float DOMINANCE_HYSTERESIS = 1.2f; // challenger must exceed the incumbent by 20%
            float panMag = fabsf(panStick);
            float tiltMag = fabsf(tiltStick);
            if (s_tiltIsDominant) {
                if (panMag > tiltMag * DOMINANCE_HYSTERESIS) s_tiltIsDominant = false;
            } else {
                if (tiltMag > panMag * DOMINANCE_HYSTERESIS) s_tiltIsDominant = true;
            }
            if (s_tiltIsDominant) {
                panStick = 0.0f;
            } else {
                tiltStick = 0.0f;
            }
        }

        panSpeedScale = constrain(panSpeedScale, 0.0f, 1.0f);
        tiltSpeedScale = constrain(tiltSpeedScale, 0.0f, 1.0f);

        float panMaxRate = panSpeedScale * GIMBAL_MAX_STEP_RATE;
        float tiltMaxRate = tiltSpeedScale * GIMBAL_MAX_STEP_RATE;
        float panTarget = clampTargetAtLimit(panStick * panMaxRate, panPositionSteps, GIMBAL_PAN_LIMIT_STEPS);

        // Bounded drift-compensation workaround (config.h) - only applied
        // while tilt is idle (not being actively commanded), so it never
        // fights or gets confused with a real tilt command, and only ever
        // does anything once tuned away from its 0.0 default. Added BEFORE
        // clampTargetAtLimit, not after - added after, it would bypass the
        // soft limit entirely and could walk the position past
        // GIMBAL_TILT_LIMIT_STEPS indefinitely with nothing to stop it.
        float rawTiltTarget = tiltStick * tiltMaxRate;
        if (tiltStick == 0.0f) {
            rawTiltTarget += TILT_IDLE_DRIFT_COMPENSATION_STEPS_S;
        }
        float tiltTarget = clampTargetAtLimit(rawTiltTarget, tiltPositionSteps, GIMBAL_TILT_LIMIT_STEPS);

        panCurrentRate = rampToward(panCurrentRate, panTarget, GIMBAL_ACCEL_STEPS_S2, dt);
        tiltCurrentRate = rampToward(tiltCurrentRate, tiltTarget, GIMBAL_ACCEL_STEPS_S2, dt);
    }

    panPositionSteps += (long)(panCurrentRate * dt);
    tiltPositionSteps += (long)(tiltCurrentRate * dt);

    // Differential mixing, RESTORED 2026-09-09 (reverts the 2026-09-07
    // "not a differential" correction - see gimbal.h for the full back-
    // and-forth). Confirmed by the user again on this date: pure tilt
    // moves motor A only while motor B stays locked; pure pan requires
    // BOTH motors to move together. That is exactly motorA = pan + tilt,
    // motorB = pan (pure tilt: motorB=0, only A moves; pure pan: A=B,
    // both move in lockstep).
    setAxisTarget(motorA, panCurrentRate + tiltCurrentRate);
    setAxisTarget(motorB, panCurrentRate);

    serviceAxis(motorA, nowUs);
    serviceAxis(motorB, nowUs);
}

float gimbalGetPanRate() { return panCurrentRate; }
float gimbalGetTiltRate() { return tiltCurrentRate; }
