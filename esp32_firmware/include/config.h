#pragma once

// ---- SBUS channel mapping (FlySky, 0-indexed) ----
// Update to match your transmitter's channel/switch assignment.
constexpr int SBUS_CH_STEER    = 0; // roll/steer stick
constexpr int SBUS_CH_THROTTLE = 1; // throttle stick
constexpr int SBUS_CH_MODE     = 4; // 2/3-pos switch: manual vs autonomous
constexpr int SBUS_CH_ARM      = 5; // 2-pos switch: disarmed vs armed (software e-stop)

// SBUS raw channel values are ~172..1811, center ~992 (11-bit channel data)
constexpr int SBUS_MIN = 172;
constexpr int SBUS_MID = 992;
constexpr int SBUS_MAX = 1811;

// Above this value on SBUS_CH_MODE -> autonomous mode; below -> manual
constexpr int SBUS_MODE_THRESHOLD = (SBUS_MIN + SBUS_MAX) / 2;
// Above this value on SBUS_CH_ARM -> armed
constexpr int SBUS_ARM_THRESHOLD = (SBUS_MIN + SBUS_MAX) / 2;

// If no valid SBUS frame arrives within this window, treat the link as lost:
// force disarmed + stop, regardless of mode.
constexpr unsigned long SBUS_TIMEOUT_MS = 500;

// ---- ESC pulse widths (microseconds) ----
// PLACEHOLDERS. Do not trust these values - they must be measured on the
// actual ESCs (CLAUDE.md Open Decisions) before any motor is enabled.
constexpr int ESC_PULSE_MIN_US     = 1000; // full reverse
constexpr int ESC_PULSE_NEUTRAL_US = 1500; // stop
constexpr int ESC_PULSE_MAX_US     = 2000; // full forward

// PCA9685 PWM frequency for analog servos/ESCs
constexpr int PCA9685_PWM_FREQ_HZ = 50;

// If no /cmd_vel message arrives within this window while in autonomous
// mode, treat it as stale and stop.
constexpr unsigned long CMD_VEL_TIMEOUT_MS = 500;

// Periodic publish rate for IMU / odom / diagnostics (milliseconds)
constexpr unsigned long PUBLISH_PERIOD_MS = 50; // 20 Hz
