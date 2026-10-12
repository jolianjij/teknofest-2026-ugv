#include "led_status.h"
#include "pins.h"
#include <Arduino.h>

// Only 2 pins / 4 static combinations for 5 states, so BOOT and FAILSAFE
// use blinking (at different rates) to stay visually distinct from the rest.
void ledStatusBegin() {
    pinMode(PIN_LED_A, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    digitalWrite(PIN_LED_A, LOW);
    digitalWrite(PIN_LED_B, LOW);
}

void ledStatusUpdate(RobotState state) {
    static unsigned long lastBlink = 0;
    static bool blinkOn = false;
    unsigned long now = millis();

    switch (state) {
        case RobotState::BOOT: // both pins blink together, slow
            if (now - lastBlink > 400) { lastBlink = now; blinkOn = !blinkOn; }
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, blinkOn);
            break;
        case RobotState::DISARMED: // A solid
            digitalWrite(PIN_LED_A, HIGH);
            digitalWrite(PIN_LED_B, LOW);
            break;
        case RobotState::MANUAL: // B solid
            digitalWrite(PIN_LED_A, LOW);
            digitalWrite(PIN_LED_B, HIGH);
            break;
        case RobotState::AUTONOMOUS: // both solid
            digitalWrite(PIN_LED_A, HIGH);
            digitalWrite(PIN_LED_B, HIGH);
            break;
        case RobotState::FAILSAFE: // A blinks fast, B off
            if (now - lastBlink > 120) { lastBlink = now; blinkOn = !blinkOn; }
            digitalWrite(PIN_LED_A, blinkOn);
            digitalWrite(PIN_LED_B, LOW);
            break;
    }
}
