#pragma once

// ESP32-WROOM-32 DevKitC pin map — Tank-Drive Obstacle Course Robot
// See docs/robot_project_plan.md and CLAUDE.md for full project context.
//
// Avoided: GPIO6-11 (internal flash), GPIO12/15 (strapping pins),
// GPIO1/3 (UART0 - reserved for the micro-ROS USB-serial transport to the Pi).
// GPIO0/2 are ALSO strapping pins but are used below for the status LED -
// see the warning on PIN_LED_A/PIN_LED_B.

// I2C bus - shared by OLED (0x3C), BNO055 (0x28), PCA9685 (0x40)
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

// PCA9685 hardware output-enable (active LOW). Driving this HIGH cuts all
// PWM outputs instantly at the hardware level, regardless of I2C/firmware
// state. Used as the firmware-level kill switch until a fully independent
// hardware e-stop loop is designed (see docs/robot_project_plan.md safety
// checklist - the competition spec requires e-stop isolated from the main
// control unit, which this alone does not satisfy).
constexpr int PIN_PCA9685_OE = 4;

// SBUS input from FlySky receiver (inverted UART, 100000 baud, 8E2).
// Own dedicated UART (UART2) - kept separate from UART0 (micro-ROS/USB).
constexpr int PIN_SBUS_RX = 16;
constexpr int PIN_SBUS_TX = 17; // unused (SBUS is receive-only here); required by the HardwareSerial API

// State LED - 2-pin bi-color LED (red on GPIO0, green on GPIO2 - confirmed wiring).
// WARNING: both are ESP32 boot-strapping pins. GPIO0 in particular must read
// HIGH at reset or the chip enters UART download mode instead of booting
// normal firmware - if boot ever becomes flaky/intermittent again, this LED
// wiring is the first thing to suspect. Safe once past boot.
constexpr int PIN_LED_A = 0;  // red
constexpr int PIN_LED_B = 2;  // green

// PCA9685 channel assignments
constexpr int PCA_CH_MOTOR_LEFT  = 0;
constexpr int PCA_CH_MOTOR_RIGHT = 1;
// Channels 2-15 free (e.g. for a future laser-designator pan/tilt) - unassigned.

// Free ESP32 GPIOs not used by this firmware, reserved for other purposes:
// 13, 14, 18, 19, 23, 27, 32, 33, and input-only 34, 35, 36, 39.
