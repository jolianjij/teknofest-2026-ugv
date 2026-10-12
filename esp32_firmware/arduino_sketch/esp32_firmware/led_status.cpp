#include "led_status.h"
#include "pins.h"
#include <Arduino.h>

// Every state blinks now - nothing sits on solid indefinitely (a vehicle
// left disarmed/idle on a bench for a while was staying lit the whole
// time). Which LED(s) blink still identifies the state; the rate is picked
// per state so adjacent states are never confused with each other, and
// every non-BOOT rate is far enough from BOOT's (400ms) and FAILSAFE's
// (120ms) that a transition doesn't read as "still booting" or "failsafe".
constexpr unsigned long BLINK_MS_BOOT       = 400;
constexpr unsigned long BLINK_MS_DISARMED   = 700;
constexpr unsigned long BLINK_MS_MANUAL     = 700;
constexpr unsigned long BLINK_MS_AUTONOMOUS = 250;
constexpr unsigned long BLINK_MS_FAILSAFE   = 120;

void ledStatusBegin() {
    pinMode(PIN_LED_A, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    digitalWrite(PIN_LED_A, LOW);
    digitalWrite(PIN_LED_B, LOW);
}

void ledStatusUpdate(RobotState state) {
    // Shared timer across all states - simple, and a transition between two
    // different rates costs at most one stale half-period, which is not
    // worth extra per-state timer bookkeeping for a status indicator.
    static unsigned long lastBlink = 0;
    static bool blinkOn = false;
    unsigned long now = millis();

    unsigned long period;
    switch (state) {
        case RobotState::BOOT:       period = BLINK_MS_BOOT; break;
        case RobotState::DISARMED:   period = BLINK_MS_DISARMED; break;
        case RobotState::MANUAL:     period = BLINK_MS_MANUAL; break;
        case RobotState::AUTONOMOUS: period = BLINK_MS_AUTONOMOUS; break;
        case RobotState::FAILSAFE:   period = BLINK_MS_FAILSAFE; break;
        case RobotState::ESTOP:      period = BLINK_MS_FAILSAFE; break;
        default:                     period = BLINK_MS_BOOT; break;
    }
    if (now - lastBlink > period) { lastBlink = now; blinkOn = !blinkOn; }

    switch (state) {
        case RobotState::BOOT: // both blink together, slow
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, blinkOn);
            break;
        case RobotState::DISARMED: // A blinks
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, LOW);
            break;
        case RobotState::MANUAL: // B blinks
            digitalWrite(PIN_LED_A, LOW);
            digitalWrite(PIN_LED_B, blinkOn);
            break;
        case RobotState::AUTONOMOUS: // both blink together, faster than BOOT
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, blinkOn);
            break;
        case RobotState::FAILSAFE: // A blinks fast, B off
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, LOW);
            break;
        case RobotState::ESTOP: // both blink together, fast - distinct from FAILSAFE's single-LED blink
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, blinkOn);
            break;
    }
}
