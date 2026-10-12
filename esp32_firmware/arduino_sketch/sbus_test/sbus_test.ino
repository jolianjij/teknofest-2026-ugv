// Standalone test - reads the FS-A8S SBUS receiver and prints all 16
// channels over USB serial. Nothing else running (no I2C, no motors).
//
// SBUS RX wired to GPIO16 (confirmed wiring, matches the main firmware).
// Open Serial Monitor at 115200 baud.

#include "sbus.h"

constexpr int PIN_SBUS_RX = 16;
constexpr int PIN_SBUS_TX = 17; // unused (SBUS is receive-only here); required by the API

static SbusReceiver sbus;

void setup() {
    Serial.begin(115200);
    delay(500);
    sbus.begin(Serial2, PIN_SBUS_RX, PIN_SBUS_TX);
    Serial.println("READY - printing SBUS channels every 200ms");
}

void loop() {
    sbus.update();

    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 200) {
        lastPrintMs = millis();

        if (sbus.failsafe()) {
            Serial.println("FAILSAFE / NO SIGNAL");
        } else {
            for (int i = 0; i < 16; i++) {
                Serial.print("ch");
                Serial.print(i);
                Serial.print("=");
                Serial.print(sbus.channel(i));
                Serial.print("  ");
            }
            Serial.println();
        }
    }
}
