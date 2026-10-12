#pragma once

// ESP32-WROOM-32 DevKitC pin map — Tank-Drive Obstacle Course Robot
// See docs/robot_project_plan.md and CLAUDE.md for full project context.
//
// Avoided: GPIO6-11 (internal flash), GPIO12/15 (strapping pins),
// GPIO1/3 (UART0 - reserved for the micro-ROS USB-serial transport to the Pi).
// GPIO0/2 are ALSO strapping pins but are used below for the status LED -
// see the warning on PIN_LED_A/PIN_LED_B.

// I2C bus - shared by OLED (0x3C), BNO055 (0x29 - this breakout's ADR pin is tied high), PCA9685 (0x40, also responds on 0x70 All Call)
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

// PCA9685 hardware output-enable (active LOW). Driving this HIGH cuts all
// PWM outputs instantly at the hardware level, regardless of I2C/firmware
// state. Used as the firmware-level kill switch until a fully independent
// hardware e-stop loop is designed (see docs/robot_project_plan.md safety
// checklist - the competition spec requires e-stop isolated from the main
// control unit, which this alone does not satisfy).
//
// CONFIRMED NOT PHYSICALLY WIRED on this build (2026-09-09, user-reported:
// "I didn't connect the OE pin of the PCA"). The firmware still drives this
// GPIO based on arm state - harmless, since it goes nowhere - but this
// specific hardware kill mechanism provides ZERO actual protection right
// now. Brake-on-disarm (relays + servos) and the physical e-stop switch
// (PIN_ESTOP) are the safety layers that actually do something on this
// vehicle as wired; do not describe this OE kill as functioning until it is
// actually connected.
constexpr int PIN_PCA9685_OE = 4;

// SBUS input from FS-A8S (FlySky) receiver - inverted UART, 100000 baud, 8E2.
// Confirmed wired to GPIO16. Own dedicated UART (UART2), kept separate from
// UART0 (micro-ROS/USB).
constexpr int PIN_SBUS_RX = 16;
constexpr int PIN_SBUS_TX = 17; // unused (SBUS is receive-only here); required by the HardwareSerial API

// State LED - 2-pin bi-color LED (red on GPIO0, green on GPIO2 - confirmed wiring).
// WARNING: both are ESP32 boot-strapping pins. GPIO0 in particular must read
// HIGH at reset or the chip enters UART download mode instead of booting
// normal firmware - if boot ever becomes flaky/intermittent again, this LED
// wiring is the first thing to suspect (a strong pull from the LED circuit
// during the reset sampling window can misread the strap). Safe once past
// boot - this only matters for the brief moment at reset, before setup() runs.
constexpr int PIN_LED_A = 0;  // red
constexpr int PIN_LED_B = 2;  // green

// Drivetrain speed: the motor driver takes a genuine ANALOG voltage
// (1V-3.3V range, not PWM) - NOT an RC-servo pulse or a PWM duty cycle.
// The classic ESP32 has exactly two pins with real DAC hardware capable of
// outputting a true analog voltage via dacWrite(): GPIO25 and GPIO26. No
// other pin can do this, PCA9685 channels included - PCA9685 only produces
// PWM, not a static analog level. See config.h for the DAC_* value
// constants and esp32_firmware.ino for how motor magnitude maps to voltage.
//
// LEFT_SPEED is GPIO25, NOT GPIO27 as originally requested - GPIO27 has no
// DAC hardware at all (only 25/26 do, full stop, regardless of what else
// is wired there), and it's also already PIN_GIMBAL_TILT_STEP below.
constexpr int PIN_MOTOR_RIGHT_SPEED_DAC = 26;
constexpr int PIN_MOTOR_LEFT_SPEED_DAC  = 25;

// 8-channel relay board (IN1-IN8) still handles direction (reverse-select)
// and braking - only the speed signal moved off PCA9685 to the DAC pins
// above. Relay board's IN1-IN8 are wired to PCA9685 channels 15 down to 8
// (descending) - CONFIRM this ordering, it's inferred from how it was
// described, not independently verified.
// Channels 0-7: spare/unassigned (0/1 formerly RIGHT/LEFT_SPEED, retired).
//
// Confirmed on the actual board (2026-09-04): each motor has its own
// reverse relay AND its own brake relay, paired consecutively:
//   right motor: IN1 reverse, IN2 brake
//   left motor : IN3 reverse, IN4 brake
constexpr int PCA_CH_RIGHT_REVERSE = 15; // IN1 - ON = right motor reverse
constexpr int PCA_CH_RIGHT_BRAKE   = 14; // IN2 - brake, right motor
constexpr int PCA_CH_LEFT_REVERSE  = 13; // IN3 - ON = left motor reverse
constexpr int PCA_CH_LEFT_BRAKE    = 12; // IN4 - brake, left motor
constexpr int PCA_CH_SPARE_1       = 11; // IN5 - unassigned, reserved
constexpr int PCA_CH_SPARE_2       = 10; // IN6 - unassigned, reserved
constexpr int PCA_CH_TANK_LEDS     = 9;  // IN7
constexpr int PCA_CH_LASER         = 8;  // IN8 - laser designator switch

// Mechanical brake servos, added 2026-09-04. These are SEPARATE from the
// electrical brake relays above (PCA_CH_RIGHT_BRAKE/LEFT_BRAKE) - the relays
// are the driver-side electrical brake, these servos are a physical/
// mechanical brake actuator per side. Engaged/released angles measured with
// brake_servo_calibration/ (a standalone sketch, not this firmware) - see
// config.h's BRAKE_SERVO_* constants. Still not driven anywhere in THIS
// firmware yet - only the calibration sketch commands them so far.
//
// Right=0, Left=7 (left moved again since - changed externally on disk from
// the previous Right=0/Left=5 - config.h/pins.h and the standalone
// brake_servo_calibration sketch must all agree on this or a given channel
// number will silently mean different servos between tools).
constexpr int PCA_CH_RIGHT_BRAKE_SERVO = 0;
constexpr int PCA_CH_LEFT_BRAKE_SERVO  = 7;

// Aiming gimbal - 2x NEMA17 + TMC2208 v1.2, STEP/DIR standalone mode,
// direct ESP32 GPIO (not PCA9685). See gimbal.h/.cpp and
// esp32_firmware/arduino_sketch/differential_gimbal_test/ for the bring-up
// this was validated against.
//
// EN is GPIO14, NOT GPIO4 - GPIO4 is PIN_PCA9685_OE above, the drivetrain's
// hardware kill switch, which the firmware actively drives based on arm
// state. Wiring the stepper EN there too means its enable state is at the
// mercy of unrelated drivetrain logic, not a dedicated level - this was
// confirmed as the actual cause of "works grounded, doesn't work on GPIO4."
//
// TILT_STEP is GPIO27, NOT GPIO17 - GPIO17 is PIN_SBUS_TX below; the UART2
// peripheral claims that pin the moment sbus.begin() runs in this firmware,
// even though SBUS itself never transmits. (GPIO23 was the first choice but
// isn't physically accessible on this board/harness.)
constexpr int PIN_GIMBAL_EN        = 14;
constexpr int PIN_GIMBAL_PAN_STEP  = 18;
constexpr int PIN_GIMBAL_PAN_DIR   = 19;
constexpr int PIN_GIMBAL_TILT_STEP = 27; // moved from the stated GPIO17 - see above
constexpr int PIN_GIMBAL_TILT_DIR  = 5;

// Emergency stop - normally-closed switch, one leg to GND, the other to this
// pin. Uses the internal pull-up (INPUT_PULLUP), so the closed/normal state
// reads LOW (grounded) and anything that opens the circuit - the switch
// being pressed, OR the wire simply breaking - reads HIGH. That second case
// is deliberate: a cut wire must fail toward "stopped", not toward "fine".
// GPIO33 (moved from the originally-picked GPIO13, per actual wiring) is a
// normal GPIO with a real internal pull-up - unlike the input-only 34-39
// pins, which have NO pull-up hardware at all and would float under
// INPUT_PULLUP - and is not a boot-strapping pin.
constexpr int PIN_ESTOP = 33;

// Free ESP32 GPIOs not used by this firmware, reserved for other purposes:
// 13, 23, 32, and input-only 34, 35, 36, 39.
