// Minimal test - one LED pin, nothing else. No I2C, no sensors, no micro-ROS.
// Purpose: confirm the board can run code at all, independent of everything else.
constexpr int PIN_LED_A = 2;

void setup() {
    pinMode(PIN_LED_A, OUTPUT);
}

void loop() {
    digitalWrite(PIN_LED_A, HIGH);
    delay(300);
    digitalWrite(PIN_LED_A, LOW);
    delay(300);
}
