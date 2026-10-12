#include <Wire.h>

// Matches the pin map in esp32_firmware/arduino_sketch/esp32_firmware/pins.h
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

void setup() {
    Serial.begin(115200);
    delay(1500);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Serial.println();
    Serial.println("I2C scanner starting (SDA=21, SCL=22)...");
}

void loop() {
    Serial.println("Scanning...");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            found++;
            Serial.print("  Found device at 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x28) Serial.print("  <- expected BNO055");
            if (addr == 0x29) Serial.print("  <- BNO055 (alt address, ADR pin high)");
            if (addr == 0x3C) Serial.print("  <- expected SSD1306 OLED");
            if (addr == 0x40) Serial.print("  <- expected PCA9685");
            Serial.println();
        }
    }
    if (found == 0) {
        Serial.println("  No devices found. Check wiring: SDA->21, SCL->22, VCC, GND, and pull-ups.");
    } else {
        Serial.print(found);
        Serial.println(" device(s) found.");
    }
    Serial.println();
    delay(3000);
}
