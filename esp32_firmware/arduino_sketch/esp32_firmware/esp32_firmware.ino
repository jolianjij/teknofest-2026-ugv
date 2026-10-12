#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

#include "pins.h"
#include "config.h"
#include "sbus.h"
#include "motor_mixer.h"
#include "oled_display.h"
#include "gimbal.h"
#include "relays.h"

// Plain text line protocol over USB serial (115200 8N1) - no micro-ROS.
// ESP32 -> Pi, once per PUBLISH_PERIOD_MS:
//   I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>\n   (orientation quaternion, gyro rad/s, accel m/s^2)
//   D,<loop_hz>,<sbus_ok 0|1>,<armed 0|1>,<autonomous_mode 0|1>,<estop_ok 0|1>\n
//     (last two fields appended - estop_ok is 1 when the e-stop circuit is
//     intact/closed, 0 when tripped or the wire is cut)
//   S,<ch0>,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>,<leftSpeedDac>,<rightSpeedDac>,<leftReverse 0|1>,<rightReverse 0|1>\n
//     (raw SBUS channels 0-5, then the actual DAC values (0-255, see config.h) + relay states being driven right now)
// Pi -> ESP32, whenever a command is issued:
//   C,<linear>,<angular>\n        (matches geometry_msgs/Twist linear.x, angular.z)
//   A,<pan>,<tilt>,<fire>\n       (autonomous aim+fire command - pan/tilt
//                                  normalized -1..1 same as a manual stick
//                                  deflection, fire is 0 or 1. Only honored
//                                  when autonomousMode && aimModeOn - see
//                                  loop(). While active, this Pi command has
//                                  FULL authority over both aiming AND the
//                                  laser relay - SwC's full position (the
//                                  manual fire control) is ignored in this
//                                  state, since the whole point is the human
//                                  isn't holding a stick to aim at that
//                                  point. The Pi-side vision node that would
//                                  actually send this doesn't exist yet;
//                                  this is just the ESP32-side hook for it.)

struct MotorPulses {
    int leftSpeedDac, rightSpeedDac;
    bool leftReverse, rightReverse;
};

static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire); // 0x29, not the 0x28 default - this breakout's ADR pin is tied high (confirmed via I2C scan)
static Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire);
static SbusReceiver sbus;

static float cmd_vel_linear = 0.0f;
static float cmd_vel_angular = 0.0f;
static unsigned long last_cmd_vel_ms = 0;

static float cmd_aim_pan = 0.0f;
static float cmd_aim_tilt = 0.0f;
static bool cmd_aim_fire = false;
static unsigned long last_cmd_aim_ms = 0;

static char rxBuf[96];
static int rxLen = 0;

static void errorLoop() {
    // LED code removed for now (was on GPIO0/2, boot-strapping pins - out of
    // the picture entirely until we revisit that wiring). Just halt.
    Serial.println("ERROR: sensor init failed, halting");
    while (true) {
        delay(1000);
    }
}

// Speed is a genuine analog voltage (1V-3.3V range) on the ESP32's two DAC
// pins (GPIO25/26 - the only pins on this chip that can do this, see
// pins.h), not PWM. Maps a 0..1 magnitude to [DAC_MIN_VALUE, DAC_MAX_VALUE]
// while actually commanded to move; 0 magnitude (or disarmed) returns
// DAC_OFF_VALUE, which on this specific driver is the SAME ~1V floor as
// DAC_MIN_VALUE, not literal 0V - confirmed on real hardware (2026-09-11)
// that this driver stops responding to any command at all once the speed
// line actually reaches 0V, so "off" and "slowest active speed" end up
// being the same voltage here even though config.h keeps them as separate
// named constants (in case a different driver board ever needs them split
// again). maxDacValue is a runtime ceiling (see VarA / driveMaxSpeedScale
// in loop()), separate from DAC_MAX_VALUE which stays the absolute
// hardware limit this can never exceed regardless of what the dial says.
static int magnitudeToDacValue(float magnitude, int maxDacValue) {
    if (magnitude <= 0.0f) return DAC_OFF_VALUE;
    if (magnitude > 1.0f) magnitude = 1.0f;
    if (maxDacValue > DAC_MAX_VALUE) maxDacValue = DAC_MAX_VALUE;
    if (maxDacValue < DAC_MIN_VALUE) maxDacValue = DAC_MIN_VALUE;
    return DAC_MIN_VALUE + (int)(magnitude * (maxDacValue - DAC_MIN_VALUE));
}

// One DAC speed pin per motor + a relay for direction + a relay for brake,
// replacing the old dual-fwd/back-channel scheme entirely (see pins.h).
//
// Confirmed on the real driver (2026-09-04): the reverse relay just shorts
// two wires to tell the driver board "go into reverse mode" - it is NOT an
// H-bridge direction pin the driver reads continuously. If the speed line
// still has voltage on it when that short changes state, the driver does
// not register the reversal at all. So direction changes must be sequenced:
// cut speed to zero, flip the relay, hold zero voltage long enough for the
// driver to settle into the new mode, THEN let speed resume - and even then,
// not instantly: speed eases back in from DAC_MIN_VALUE up to whatever's
// commanded over REVERSAL_RAMP_MS (config.h), rather than snapping straight
// to full torque the moment the hold ends. Explicit request (2026-09-11):
// brake the motor, then bring it back up gently, not abruptly.
//
// s_lastLeftReverse / s_lastRightReverse track the direction the relay is
// CURRENTLY holding. On a genuine flip (only while actually driving - a
// zero-to-zero crossing needs no sequencing) the relay is switched to the
// new direction IMMEDIATELY, in this same loop iteration, and s_last is
// updated to match in that same step - so "last" and "applied" are never
// out of sync across iterations. Only the SPEED is held at zero for a
// separate timer afterward. An earlier version held the relay at the OLD
// direction during this window instead, which left s_last permanently
// stale (it only updated once the hold ended, which never happened while
// the stick stayed in the new direction) - the relay could never actually
// flip for as long as the stick was held past centre. Updating the relay
// and s_last together, on the spot, is what avoids that.
static bool s_lastLeftReverse = false;
static bool s_lastRightReverse = false;
static uint32_t s_leftSpeedHoldUntilMs = 0;
static uint32_t s_rightSpeedHoldUntilMs = 0;

// Ramp-up state: once a side's post-reversal hold ends, its speed eases in
// from DAC_MIN_VALUE up to the commanded value over REVERSAL_RAMP_MS instead
// of jumping straight there (see REVERSAL_RAMP_MS, config.h). s_*WasHeld
// tracks the hold's own state so the ramp start is only latched once, right
// on the falling edge, not re-latched every loop while already ramping.
static bool s_leftWasHeld = false;
static bool s_rightWasHeld = false;
static uint32_t s_leftRampStartMs = 0;
static uint32_t s_rightRampStartMs = 0;

// Minimum-reverse-commitment (2026-09-11, explicit request: "you can do
// reverse but it needs not to be a very small reverse, at least one
// second of reverse"). Once a wheel starts reversing - whether from an
// intentional backup or a tight steering-driven pivot - it must keep
// reversing for at least MIN_REVERSE_COMMIT_MS before it is allowed to
// flip back to forward. This is what actually stops noisy steering error
// from oscillating a wheel between directions (replacing an earlier,
// blunter fix in motor_mixer.cpp's history that banned reverse-from-
// steering entirely and cost the vehicle its ability to pivot tighter
// than TRACK_SEPARATION_M's radius). Separate from, and layered
// UNDERNEATH, the existing DIRECTION_REVERSAL_BRAKE_MS/REVERSAL_RAMP_MS
// handling below - this decides WHETHER a direction flip is allowed to
// happen at all; that mechanism handles HOW a flip that IS allowed gets
// executed electrically (brake, hold, ramp).
static bool s_leftReverseCommitted = false;
static uint32_t s_leftReverseCommitUntilMs = 0;
static bool s_rightReverseCommitted = false;
static uint32_t s_rightReverseCommitUntilMs = 0;

// Reverse impact guard state - see REVERSE_IMPACT_GYRO_DELTA's comment
// (config.h) for the full reasoning. s_hasGyroBaseline is deliberately
// dropped (not carried over) the moment neither side is reversing, so the
// next time reversing begins gets a fresh baseline seeded from that first
// sample rather than diffing against gyro data that could be seconds
// stale.
static bool s_hasGyroBaseline = false;
static float s_lastGyroMag = 0.0f;

static bool checkReverseImpactGuard(Adafruit_BNO055 &imu) {
    imu::Vector<3> gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float mag = sqrtf(gyro.x() * gyro.x() + gyro.y() * gyro.y() + gyro.z() * gyro.z());
    bool impact = false;
    if (s_hasGyroBaseline) {
        float delta = fabsf(mag - s_lastGyroMag);
        impact = delta > REVERSE_IMPACT_GYRO_DELTA;
    }
    s_lastGyroMag = mag;
    s_hasGyroBaseline = true;
    return impact;
}

static bool applyReverseCommitment(bool rawReverse, bool &committed, uint32_t &commitUntilMs, uint32_t now) {
    if (!committed) {
        if (rawReverse) {
            committed = true;
            commitUntilMs = now + MIN_REVERSE_COMMIT_MS;
        }
        return rawReverse;
    }
    // Still within the committed window, or the raw signal still wants
    // reverse even after the window elapsed - either way, keep reversing.
    // Only release once BOTH the minimum time has passed AND the signal
    // has actually moved on, so "at least one second" is measured from
    // when reverse began, not reset by re-checking.
    bool timeUp = (int32_t)(now - commitUntilMs) >= 0;
    if (timeUp && !rawReverse) {
        committed = false;
        return false;
    }
    return true;
}

// Drives a brake servo to an angle. Same pulse-width math as the standalone
// brake_servo_calibration sketch used to measure BRAKE_SERVO_*_DEG - keep
// the two in agreement, since a mismatch here would make "0 = engaged"
// silently stop being true.
static void setBrakeServoAngle(int channel, float angleDeg) {
    // Same "skip the I2C write if nothing changed" reasoning as
    // pcaDigitalWrite (relays.h) - this is called unconditionally every
    // loop() iteration for both servos, and was contributing to the same
    // loop-timing problem that was starving the gimbal steppers.
    static float lastAngle[16];
    static bool haveLast[16] = { false };
    if (channel < 0 || channel >= 16) return;
    if (haveLast[channel] && fabsf(lastAngle[channel] - angleDeg) < 0.5f) return;
    lastAngle[channel] = angleDeg;
    haveLast[channel] = true;

    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    float pulseUs = BRAKE_SERVO_PULSE_MIN_US +
        (angleDeg / 180.0f) * (BRAKE_SERVO_PULSE_MAX_US - BRAKE_SERVO_PULSE_MIN_US);
    float periodUs = 1000000.0f / PCA9685_PWM_FREQ_HZ;
    pca.setPWM(channel, 0, (int)(pulseUs / periodUs * 4096.0f + 0.5f));
}

static MotorPulses applyMotorOutputs(const MotorOutputs &m, bool armed,
                                      bool rightBrakeChannelActive, bool leftBrakeChannelActive,
                                      float leftBrakeServoProportional, int maxDacValue) {
    digitalWrite(PIN_PCA9685_OE, armed ? LOW : HIGH); // hardware-level cut when disarmed

    const uint32_t now = millis();

    MotorPulses p{DAC_OFF_VALUE, DAC_OFF_VALUE, false, false};
    if (armed) {
        // Minimum-commitment gate (see applyReverseCommitment above) - the
        // raw m.left/m.right < 0 decision is what steering/backup actually
        // wants; the commitment layer only controls WHEN a flip in or out
        // of reverse is allowed to take effect.
        p.leftReverse = applyReverseCommitment(m.left < 0.0f, s_leftReverseCommitted, s_leftReverseCommitUntilMs, now);
        p.rightReverse = applyReverseCommitment(m.right < 0.0f, s_rightReverseCommitted, s_rightReverseCommitUntilMs, now);
        // Reverse always commands full magnitude (within maxDacValue, the
        // runtime speed-dial ceiling - this does not bypass that), never a
        // proportional fraction of the stick/cmd_vel - explicit request
        // (2026-09-11): "always reverse on max speed in all cases in auto
        // and manual because reverse is slow and fixed speed" - i.e. real
        // reverse speed on this driver doesn't scale with commanded
        // voltage the way forward does, so a partial command in reverse
        // just wastes the (already slow) reverse capability for no benefit.
        // Applies identically in AUTONOMOUS and MANUAL since both funnel
        // through this one function. Trade-off worth knowing: this removes
        // fine proportional control specifically on a wheel that is
        // reversing - e.g. an autonomous differential turn that wants a
        // small reverse nudge on one side gets full reverse there instead.
        float leftMagnitude = p.leftReverse ? 1.0f : fabsf(m.left);
        float rightMagnitude = p.rightReverse ? 1.0f : fabsf(m.right);
        p.leftSpeedDac = magnitudeToDacValue(leftMagnitude, maxDacValue);
        p.rightSpeedDac = magnitudeToDacValue(rightMagnitude, maxDacValue);

        // Reverse impact guard (see REVERSE_IMPACT_GYRO_DELTA, config.h) -
        // only checked while actually reversing, both to bound the extra
        // I2C cost to exactly the windows where it matters and because
        // "backed into a wall" is only a meaningful question while backing
        // up at all. On a detected impact: stop immediately (don't know
        // which side hit, so zero and brake both), and release the reverse
        // commitment early - continuing to satisfy MIN_REVERSE_COMMIT_MS's
        // "keep reversing at least a second" rule is never correct once a
        // collision has actually been detected, safety overrides that
        // timer outright.
        if ((p.leftReverse || p.rightReverse) && checkReverseImpactGuard(bno)) {
            p.leftSpeedDac = DAC_OFF_VALUE;
            p.rightSpeedDac = DAC_OFF_VALUE;
            s_leftReverseCommitted = false;
            s_rightReverseCommitted = false;
            s_leftSpeedHoldUntilMs = now + REVERSE_IMPACT_BRAKE_HOLD_MS;
            s_rightSpeedHoldUntilMs = now + REVERSE_IMPACT_BRAKE_HOLD_MS;
        }
    }
    // Baseline only means anything while actually reversing - dropped
    // (not carried over) the instant that stops being true, whether from
    // disarming or simply returning to forward/neutral, so the next
    // reversing spell always seeds fresh rather than diffing against
    // gyro data that could be stale by seconds.
    if (!armed || !(p.leftReverse || p.rightReverse)) {
        s_hasGyroBaseline = false;
    }

    // A flip only counts when that side is actually being driven - crossing
    // zero while already stopped needs no sequencing, there is no live
    // voltage for the driver to misread. Verified 2026-09-11 (explicit
    // request: "you can use the reverse only if the motor is braked"):
    // since reverse always commands max magnitude, p.leftSpeedDac is >
    // DAC_OFF_VALUE on every single reverse entry with no exception - so
    // in practice this check now ALWAYS fires for a genuine reverse flip,
    // including straight from a dead stop, not just mid-drive. That is
    // the correct, safer behavior: every reverse entry unconditionally
    // gets the brake+hold below before any real reverse torque is
    // applied, with no path that skips it.
    const bool leftFlip  = (p.leftReverse  != s_lastLeftReverse)  && (p.leftSpeedDac  > DAC_OFF_VALUE);
    const bool rightFlip = (p.rightReverse != s_lastRightReverse) && (p.rightSpeedDac > DAC_OFF_VALUE);

    if (leftFlip)  s_leftSpeedHoldUntilMs  = now + DIRECTION_REVERSAL_BRAKE_MS;
    if (rightFlip) s_rightSpeedHoldUntilMs = now + DIRECTION_REVERSAL_BRAKE_MS;

    // Relay state is applied and latched THIS iteration regardless of the
    // hold - only speed is gated below.
    s_lastLeftReverse = p.leftReverse;
    s_lastRightReverse = p.rightReverse;

    const bool leftSpeedHeld  = (int32_t)(s_leftSpeedHoldUntilMs  - now) > 0;
    const bool rightSpeedHeld = (int32_t)(s_rightSpeedHoldUntilMs - now) > 0;
    if (leftSpeedHeld)  p.leftSpeedDac  = DAC_OFF_VALUE;
    if (rightSpeedHeld) p.rightSpeedDac = DAC_OFF_VALUE;

    // Falling edge of the hold latches the ramp start exactly once - not
    // every loop while already past it.
    if (s_leftWasHeld && !leftSpeedHeld)   s_leftRampStartMs  = now;
    if (s_rightWasHeld && !rightSpeedHeld) s_rightRampStartMs = now;
    s_leftWasHeld = leftSpeedHeld;
    s_rightWasHeld = rightSpeedHeld;

    if (!leftSpeedHeld && p.leftSpeedDac > DAC_MIN_VALUE) {
        uint32_t elapsed = now - s_leftRampStartMs;
        if (elapsed < REVERSAL_RAMP_MS) {
            float frac = (float)elapsed / (float)REVERSAL_RAMP_MS;
            p.leftSpeedDac = DAC_MIN_VALUE + (int)(frac * (p.leftSpeedDac - DAC_MIN_VALUE));
        }
    }
    if (!rightSpeedHeld && p.rightSpeedDac > DAC_MIN_VALUE) {
        uint32_t elapsed = now - s_rightRampStartMs;
        if (elapsed < REVERSAL_RAMP_MS) {
            float frac = (float)elapsed / (float)REVERSAL_RAMP_MS;
            p.rightSpeedDac = DAC_MIN_VALUE + (int)(frac * (p.rightSpeedDac - DAC_MIN_VALUE));
        }
    }

    dacWrite(PIN_MOTOR_LEFT_SPEED_DAC, p.leftSpeedDac);
    dacWrite(PIN_MOTOR_RIGHT_SPEED_DAC, p.rightSpeedDac);
    pcaDigitalWrite(pca, PCA_CH_LEFT_REVERSE, p.leftReverse);
    pcaDigitalWrite(pca, PCA_CH_RIGHT_REVERSE, p.rightReverse);

    // Brake engages automatically when disarmed (BRAKE_ON_DISARM, config.h -
    // both sides together), on the per-side manual channel
    // (SBUS_CH_RIGHT_BRAKE / SBUS_CH_LEFT_BRAKE_SERVO, see config.h - moved
    // off the old shared SwD switch 2026-09-09), or on that side's own
    // direction-change hold, so the track isn't coasting under its own
    // momentum with the speed line forced to zero and no drive torque.
    const bool autoBrake = BRAKE_ON_DISARM && !armed;
    const bool rightBrakeEngaged = autoBrake || rightBrakeChannelActive || rightSpeedHeld;
    const bool leftBrakeEngaged  = autoBrake || leftBrakeChannelActive  || leftSpeedHeld;
    pcaDigitalWrite(pca, PCA_CH_RIGHT_BRAKE, rightBrakeEngaged);
    pcaDigitalWrite(pca, PCA_CH_LEFT_BRAKE,  leftBrakeEngaged);

    // Mechanical brake servos mirror the manual/disarm brake condition ONLY -
    // deliberately EXCLUDING the direction-reversal hold (removed 2026-09-09,
    // explicit request: "remove the mechanical brake when reverse" - see the
    // comment on BRAKE_SERVO_* in config.h for why). The electrical relay
    // above still includes the reversal hold; that's what actually makes
    // reversal work, unaffected by this.
    const bool rightServoEngaged = autoBrake || rightBrakeChannelActive;
    setBrakeServoAngle(PCA_CH_RIGHT_BRAKE_SERVO,
        rightServoEngaged ? BRAKE_SERVO_RIGHT_ENGAGED_DEG : BRAKE_SERVO_RIGHT_RELEASED_DEG);

    // Left servo gets genuine proportional control from
    // SBUS_CH_LEFT_BRAKE_SERVO (0..1, already deadzoned by the caller)
    // instead of a hard on/off snap - autoBrake still forces it fully
    // engaged regardless of the channel, since that's a safety default, not
    // a tuning knob the operator should be able to override by centering a
    // stick.
    float leftServoTarget = autoBrake
        ? BRAKE_SERVO_LEFT_ENGAGED_DEG
        : BRAKE_SERVO_LEFT_RELEASED_DEG + leftBrakeServoProportional *
              (BRAKE_SERVO_LEFT_ENGAGED_DEG - BRAKE_SERVO_LEFT_RELEASED_DEG);
    setBrakeServoAngle(PCA_CH_LEFT_BRAKE_SERVO, leftServoTarget);

    return p;
}

static float sbusToNormalized(uint16_t raw) {
    float v = (float)(raw - SBUS_MID) / (float)(SBUS_MAX - SBUS_MID);
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

// Zeroes out anything within deadzone of center - without this, physical
// stick center drift (a stick reading e.g. 0.08 when "at rest") passes
// straight through as a real command. Confirmed necessary: measured 1.19V
// on the motor driver with the stick centered before this was added.
static float applyDeadzone(float v, float deadzone) {
    return (fabsf(v) < deadzone) ? 0.0f : v;
}

// Simple 2-color status LED: red = autonomous, green = manual, off otherwise.
// Runs on GPIO0/2 - safe during normal operation (see pins.h warning; only
// the boot-time reset window is risky, and this never touches the pins
// before setup() has already gotten past that).
static void updateLed(RobotState state) {
    bool red = (state == RobotState::AUTONOMOUS);
    bool green = (state == RobotState::MANUAL);
    digitalWrite(PIN_LED_A, red ? HIGH : LOW);
    digitalWrite(PIN_LED_B, green ? HIGH : LOW);
}

// Parses one complete line already in rxBuf (NUL-terminated, no trailing \n).
static void handleLine(const char *line) {
    if (line[0] == 'C' && line[1] == ',') {
        float lin, ang;
        if (sscanf(line + 2, "%f,%f", &lin, &ang) == 2) {
            cmd_vel_linear = lin;
            cmd_vel_angular = ang;
            last_cmd_vel_ms = millis();
        }
    } else if (line[0] == 'A' && line[1] == ',') {
        float pan, tilt, fire;
        if (sscanf(line + 2, "%f,%f,%f", &pan, &tilt, &fire) == 3) {
            cmd_aim_pan = pan;
            cmd_aim_tilt = tilt;
            cmd_aim_fire = fire > 0.5f;
            last_cmd_aim_ms = millis();
        }
    }
}

static void pollIncomingSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rxBuf[rxLen] = '\0';
            handleLine(rxBuf);
            rxLen = 0;
        } else if (c != '\r' && rxLen < (int)sizeof(rxBuf) - 1) {
            rxBuf[rxLen++] = c;
        }
    }
}

static void publishSensors(bool armed, bool autonomousMode, bool estopOk, const MotorPulses &pulses) {
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion q = bno.getQuat();

    Serial.print("I,");
    Serial.print(q.w(), 4); Serial.print(",");
    Serial.print(q.x(), 4); Serial.print(",");
    Serial.print(q.y(), 4); Serial.print(",");
    Serial.print(q.z(), 4); Serial.print(",");
    Serial.print(gyro.x(), 4); Serial.print(",");
    Serial.print(gyro.y(), 4); Serial.print(",");
    Serial.print(gyro.z(), 4); Serial.print(",");
    Serial.print(accel.x(), 4); Serial.print(",");
    Serial.print(accel.y(), 4); Serial.print(",");
    Serial.println(accel.z(), 4);

    static unsigned long lastLoopMs = 0;
    unsigned long now = millis();
    float loopHz = lastLoopMs > 0 ? 1000.0f / (float)(now - lastLoopMs) : 0.0f;
    lastLoopMs = now;

    // Additive: appended autonomous_mode and estop_ok after the original
    // three fields so any existing consumer reading only the first three
    // is unaffected.
    Serial.print("D,");
    Serial.print(loopHz, 1); Serial.print(",");
    Serial.print(sbus.failsafe() ? 0 : 1); Serial.print(",");
    Serial.print(armed ? 1 : 0); Serial.print(",");
    Serial.print(autonomousMode ? 1 : 0); Serial.print(",");
    Serial.println(estopOk ? 1 : 0);

    Serial.print("S,");
    for (int i = 0; i < 6; i++) {
        Serial.print(sbus.channel(i)); Serial.print(",");
    }
    Serial.print(pulses.leftSpeedDac); Serial.print(",");
    Serial.print(pulses.rightSpeedDac); Serial.print(",");
    Serial.print(pulses.leftReverse ? 1 : 0); Serial.print(",");
    Serial.println(pulses.rightReverse ? 1 : 0);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, HIGH); // disabled until armed

    pinMode(PIN_LED_A, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    digitalWrite(PIN_LED_A, LOW);
    digitalWrite(PIN_LED_B, LOW);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    // 400kHz Fast Mode instead of the 100kHz default - BNO055, PCA9685, and
    // the SSD1306 OLED all support it, and every transaction on this bus
    // (there are many, every loop iteration) now costs a quarter as long.
    // Part of fixing the gimbal steppers being starved of loop time.
    Wire.setClock(400000);

    pinMode(PIN_ESTOP, INPUT_PULLUP);

    oledBegin();

    if (!bno.begin()) {
        errorLoop(); // IMU is safety-relevant (heading for EKF) - do not run without it
    }
    bno.setExtCrystalUse(true);

    pca.begin();
    delay(500);
    pca.setPWMFreq(PCA9685_PWM_FREQ_HZ);
    // Known-safe relay state before anything can be armed: brakes ON,
    // everything else OFF.
    pcaDigitalWrite(pca, PCA_CH_LEFT_REVERSE, false);
    pcaDigitalWrite(pca, PCA_CH_RIGHT_REVERSE, false);
    pcaDigitalWrite(pca, PCA_CH_LEFT_BRAKE, true);
    pcaDigitalWrite(pca, PCA_CH_RIGHT_BRAKE, true);
    pcaDigitalWrite(pca, PCA_CH_TANK_LEDS, false);
    pcaDigitalWrite(pca, PCA_CH_LASER, false);

    gimbalBegin();

    sbus.begin(Serial2, PIN_SBUS_RX, PIN_SBUS_TX);

    Serial.println("READY");
}

void loop() {
    sbus.update();
    pollIncomingSerial();

    // Hardware e-stop: normally-closed switch to GND on PIN_ESTOP (pins.h),
    // read via the internal pull-up. Closed (normal) = LOW. Open - pressed,
    // OR the wire simply breaking - = HIGH, and either one must stop the
    // vehicle. This is checked before, and overrides, every other control
    // path (SBUS arm switch included): folding it into `armed` here means
    // the drivetrain's existing PCA9685 !OE hardware kill, brake-on-disarm,
    // and the gimbal's own `armed`-gated enable all already cut out
    // correctly with no further changes needed anywhere else.
    bool estopOk = digitalRead(PIN_ESTOP) == LOW;

    bool sbusOk = !sbus.failsafe();
    bool armed = estopOk && sbusOk && sbusToNormalized(sbus.channel(SBUS_CH_ARM)) > 0.0f;
    // Reverted 2026-09-11: autonomous is selected ONLY by the physical SwB
    // switch, per explicit request ("I want autonomous turn on to be just
    // the switch, nothing from the dashboard"). The dashboard-requested
    // second path (M, command, cmd_mode_autonomous) that briefly existed
    // has been removed entirely, not just disabled.
    bool autonomousMode = sbusOk && sbus.channel(SBUS_CH_MODE) > SBUS_MODE_THRESHOLD;

    // Per-side manual brake channels (2026-09-09, replaces the old shared
    // SwD brake switch - see config.h). Negative deflection is clamped to
    // "no brake" (0.0), not "negative brake" - there is no such thing, and
    // a resting stick reads near 0 either way. leftBrakeServoProportional
    // stays a continuous 0..1 value (used directly by the servo); the
    // *ChannelActive booleans are what the relays actually react to.
    float rightBrakeProportional = sbusOk
        ? constrain(applyDeadzone(sbusToNormalized(sbus.channel(SBUS_CH_RIGHT_BRAKE)), BRAKE_CHANNEL_DEADZONE), 0.0f, 1.0f)
        : 0.0f;
    float leftBrakeServoProportional = sbusOk
        ? constrain(applyDeadzone(sbusToNormalized(sbus.channel(SBUS_CH_LEFT_BRAKE_SERVO)), BRAKE_CHANNEL_DEADZONE), 0.0f, 1.0f)
        : 0.0f;
    bool rightBrakeChannelActive = rightBrakeProportional > 0.0f;
    bool leftBrakeChannelActive = leftBrakeServoProportional > 0.0f;
    // Each side's brake gates ONLY that side's own drive output (below) -
    // fixed 2026-09-11, was previously ORed into one combined flag that
    // zeroed BOTH wheels' drive command the instant either brake channel
    // was active, even though the brake relay/servo logic below this point
    // was already correctly per-side. Reported directly: "when I brake one
    // motor don't shut power for both, we need individual brakes."

    // SwD now drives the tank LEDs relay directly as a manual switch
    // (2026-09-09), replacing the old "lit while driving" automatic
    // heuristic entirely.
    bool tankLedSwitch = sbusOk && sbusToNormalized(sbus.channel(SBUS_CH_TANK_LEDS)) > 0.0f;

    // SwC: 3-position aim switch. Off = drive normally. Middle or full =
    // aim mode (drive forced to zero, steer/throttle become pan/tilt
    // direction). In MANUAL, SwC's full position additionally fires the
    // laser directly (human-controlled). In AUTONOMOUS, firing instead
    // comes entirely from the Pi's 'A,' command regardless of middle vs
    // full - see the laserOn computation below and the header comment.
    int aimSwitchRaw = sbus.channel(SBUS_CH_AIM_SWITCH);
    bool aimModeOn = sbusOk && aimSwitchRaw > SBUS_3POS_LOW_THRESHOLD;
    bool manualFireSwitch = sbusOk && aimSwitchRaw > SBUS_3POS_HIGH_THRESHOLD;

    RobotState state;
    MotorOutputs outputs{0.0f, 0.0f};

    if (!estopOk) {
        state = RobotState::ESTOP;
    } else if (!sbusOk || !armed) {
        state = sbusOk ? RobotState::DISARMED : RobotState::FAILSAFE;
    } else if (autonomousMode) {
        state = RobotState::AUTONOMOUS;
        // Aim mode blocks driving entirely (both sides) - the shooting
        // phase stops the robot moving. Brake is per-side: braking one
        // wheel zeroes only that wheel's drive command below, leaving the
        // other free to keep moving (e.g. a deliberate pivot, or a manual
        // brake override on one side while the other continues on cmd_vel).
        if (!aimModeOn) {
            bool cmdFresh = (millis() - last_cmd_vel_ms) < CMD_VEL_TIMEOUT_MS;
            outputs = cmdFresh ? mixDifferential(cmd_vel_linear, cmd_vel_angular)
                                : MotorOutputs{0.0f, 0.0f};
            if (rightBrakeChannelActive) outputs.right = 0.0f;
            if (leftBrakeChannelActive)  outputs.left  = 0.0f;
        }
    } else {
        state = RobotState::MANUAL;
        // Drive only when aim mode is off (aim mode blocks both sides, same
        // as autonomous above). Each side's own brake channel independently
        // zeroes only that stick's output - braking right does not touch
        // left, and vice versa.
        //
        // Tank mix: each stick drives its own track directly. No
        // mixDifferential() here - that stays reserved for the AUTONOMOUS
        // branch above, which genuinely receives linear/angular (Nav2's
        // /cmd_vel), a different representation that actually needs mixing.
        if (!aimModeOn) {
            outputs.right = rightBrakeChannelActive ? 0.0f
                : applyDeadzone(sbusToNormalized(sbus.channel(SBUS_CH_RIGHT_STICK)), DRIVE_STICK_DEADZONE);
            outputs.left = leftBrakeChannelActive ? 0.0f
                : applyDeadzone(sbusToNormalized(sbus.channel(SBUS_CH_LEFT_STICK)), DRIVE_STICK_DEADZONE);
        }
    }

    // VarA doubles as the drive-speed ceiling here and as the gimbal pan
    // speed dial further down (SBUS_CH_PAN_SPEED_KNOB) - the two never
    // apply at the same time, since aim mode forces drive to zero and
    // normal driving forces the gimbal stick-reuse path off. Full
    // counter-clockwise pins top speed at DAC_MIN_VALUE (barely moves);
    // full clockwise allows all the way to DAC_MAX_VALUE.
    float driveMaxSpeedScale = (sbusToNormalized(sbus.channel(SBUS_CH_PAN_SPEED_KNOB)) + 1.0f) / 2.0f;
    int driveMaxDac = DAC_MIN_VALUE + (int)(driveMaxSpeedScale * (DAC_MAX_VALUE - DAC_MIN_VALUE));

    MotorPulses pulses = applyMotorOutputs(outputs, armed && sbusOk,
        rightBrakeChannelActive, leftBrakeChannelActive, leftBrakeServoProportional, driveMaxDac);

    // Aiming pan/tilt. In MANUAL, reuses the steer/throttle sticks as
    // direction inputs while aim mode is on. In AUTONOMOUS, direction comes
    // from the Pi's 'A,' command instead (see header comment) - the actual
    // vision-based targeting that would send this doesn't exist yet, this
    // is just the firmware-side hook for it. Either way, brake does NOT
    // gate this - braking in place shouldn't disable aiming. Rate control
    // (see gimbal.h): holds still, not last-position, whenever the stick/
    // command is centered or the gate is false - open-loop steppers have
    // no "last position" to return to the way a servo naturally would.
    bool gimbalActive = (armed && sbusOk && aimModeOn &&
                          (state == RobotState::MANUAL || state == RobotState::AUTONOMOUS));
    bool autonomousAimActive = gimbalActive && state == RobotState::AUTONOMOUS;
    bool aimCmdFresh = (millis() - last_cmd_aim_ms) < CMD_VEL_TIMEOUT_MS;
    float panStick = 0.0f;
    float tiltStick = 0.0f;
    if (gimbalActive && state == RobotState::MANUAL) {
        panStick = sbusToNormalized(sbus.channel(SBUS_CH_RIGHT_STICK));
        // Negated - stick down was moving tilt up, confirmed backwards on hardware.
        tiltStick = -sbusToNormalized(sbus.channel(SBUS_CH_LEFT_STICK));
    } else if (autonomousAimActive && aimCmdFresh) {
        panStick = cmd_aim_pan;
        tiltStick = cmd_aim_tilt;
        // else (stale/no command yet): stays centered (holds still) - same
        // staleness convention as cmd_vel.
    }
    // VarA/VarB are speed dials, not direction - map their -1..1 range to
    // 0..1 (full CCW = 0% speed, full CW = 100%).
    float panSpeedScale = (sbusToNormalized(sbus.channel(SBUS_CH_PAN_SPEED_KNOB)) + 1.0f) / 2.0f;
    float tiltSpeedScale = (sbusToNormalized(sbus.channel(SBUS_CH_TILT_SPEED_KNOB)) + 1.0f) / 2.0f;
    gimbalUpdate(panStick, tiltStick, panSpeedScale, tiltSpeedScale, gimbalActive);

    // Laser: in autonomous aim mode, the Pi's fire command has FULL
    // authority (fresh command required, defaults to off if stale/absent -
    // fails safe). Otherwise (manual, or autonomous but not in an aim
    // position), it's the operator's direct switch control.
    bool laserOn = autonomousAimActive ? (aimCmdFresh && cmd_aim_fire) : manualFireSwitch;

    // TEMPORARY diagnostic for the "does the speed dial/accel ramp actually
    // do anything" question - watch this in the Serial Monitor while
    // turning VarA/VarB and holding the stick over: panSpeedScale/
    // tiltSpeedScale should visibly track the dial (0..1), and
    // panRate/tiltRate should climb gradually toward the stick-commanded
    // target instead of jumping straight there. Remove once confirmed working.
    static unsigned long lastGimbalDebugMs = 0;
    if (millis() - lastGimbalDebugMs >= 200) {
        lastGimbalDebugMs = millis();
        Serial.print("G,panSpeedScale="); Serial.print(panSpeedScale, 2);
        Serial.print(",tiltSpeedScale="); Serial.print(tiltSpeedScale, 2);
        Serial.print(",panRate="); Serial.print(gimbalGetPanRate(), 1);
        Serial.print(",tiltRate="); Serial.println(gimbalGetTiltRate(), 1);
    }

    pcaDigitalWrite(pca, PCA_CH_LASER, laserOn && armed && sbusOk);

    // SwD now drives this directly as a manual switch (2026-09-09) -
    // replaces the earlier "lit while driving" heuristic (which was tied to
    // `pulses`, the actual post-brake/post-reversal-hold DAC values, so the
    // lights tracked whether the wheels were genuinely being told to spin
    // rather than just whether the vehicle was armed) - that automatic
    // behavior is gone now, not layered underneath the switch.
    pcaDigitalWrite(pca, PCA_CH_TANK_LEDS, tankLedSwitch);

    updateLed(state);

    static unsigned long lastPublishMs = 0;
    if (millis() - lastPublishMs >= PUBLISH_PERIOD_MS) {
        lastPublishMs = millis();
        publishSensors(armed && sbusOk, autonomousMode, estopOk, pulses);
    }

    // Throttled to the OLED's own ~5Hz refresh rate (oled_display.cpp) - this
    // used to run every loop() iteration unconditionally, which meant a full
    // BNO055 I2C read every iteration for a display that only redraws 5
    // times a second. That wasted I2C bus time was part of what was
    // starving the gimbal steppers of loop time (see relays.h/gimbal.cpp).
    static unsigned long lastOledMs = 0;
    if (millis() - lastOledMs >= 200) {
        lastOledMs = millis();
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        oledUpdate(state, euler.x(), sbusOk, true);
    }
}
