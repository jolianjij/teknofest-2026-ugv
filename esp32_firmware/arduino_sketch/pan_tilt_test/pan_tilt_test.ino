// Standalone pan/tilt servo test via PCA9685 - nothing else running.
// Pan = PCA9685 channel 6, Tilt = PCA9685 channel 7.
//
// Open the Arduino IDE Serial Monitor at 115200 baud, line ending "Newline",
// and send commands:
//   P,<microseconds>     e.g. P,1500   -> sets pan
//   T,<microseconds>     e.g. T,1800   -> sets tilt
//   C                    -> centers both (1500us)
//
// Pulse range defaults to 500-2500us (common hobby servo range). If your
// servo buzzes/stalls at the extremes, narrow SERVO_MIN_US/SERVO_MAX_US.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

constexpr int PCA_CH_PAN = 6;
constexpr int PCA_CH_TILT = 7;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_CENTER_US = 1500;

static Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire);

static char rxBuf[32];
static int rxLen = 0;

static int clampUs(int us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

static void setPan(int us) {
    us = clampUs(us);
    pca.writeMicroseconds(PCA_CH_PAN, us);
    Serial.print("PAN -> "); Serial.println(us);
}

static void setTilt(int us) {
    us = clampUs(us);
    pca.writeMicroseconds(PCA_CH_TILT, us);
    Serial.print("TILT -> "); Serial.println(us);
}

static void handleLine(const char *line) {
    if (line[0] == 'P' && line[1] == ',') {
        setPan(atoi(line + 2));
    } else if (line[0] == 'T' && line[1] == ',') {
        setTilt(atoi(line + 2));
    } else if (line[0] == 'C') {
        setPan(SERVO_CENTER_US);
        setTilt(SERVO_CENTER_US);
    } else {
        Serial.print("unrecognized: "); Serial.println(line);
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    pca.begin();
    pca.setPWMFreq(50);

    setPan(SERVO_CENTER_US);
    setTilt(SERVO_CENTER_US);

    Serial.println("READY - send P,<us> / T,<us> / C (center)");
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rxBuf[rxLen] = '\0';
            if (rxLen > 0) handleLine(rxBuf);
            rxLen = 0;
        } else if (c != '\r' && rxLen < (int)sizeof(rxBuf) - 1) {
            rxBuf[rxLen++] = c;
        }
    }
}
