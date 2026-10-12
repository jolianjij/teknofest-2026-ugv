// Standalone tool for finding the engaged/released angles of the two
// mechanical brake servos. NOT part of the main firmware - flash this
// sketch by itself, on the bench, with the vehicle able to move freely
// (wheels off the ground) so you can watch each brake mechanism actuate.
//
// Right brake servo = PCA9685 channel 0
// Left  brake servo = PCA9685 channel 7
// (see pins.h in the main firmware - PCA_CH_RIGHT_BRAKE_SERVO / _LEFT_ -
// keep these two constants in agreement with pins.h or a channel number will
// silently mean different servos between this tool and the main firmware)
//
// Usage: open the Serial Monitor at 115200 baud, line ending "Newline".
// Type a command and press Enter:
//   R90        -> right brake servo to 90 degrees
//   L45        -> left brake servo to 45 degrees
//   R120 L60   -> both in one line, space-separated
//   ?          -> print this help again
//
// Move each servo through its range in small steps (e.g. 10 degrees at a
// time) and watch the physical brake mechanism. Note the angle where it
// FIRST fully engages and the angle where it is FULLY released with no
// drag - those two numbers per side are what config.h needs
// (BRAKE_SERVO_RIGHT_ENGAGED_DEG etc.) to wire this into the main firmware.
//
// Nothing here assumes an angle is safe - it starts idle (no channel driven)
// until you send a command, so a wrong first guess can't slam anything.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
int SERVO_PWM_FREQ_HZ = 50;
constexpr int PCA_CH_RIGHT_BRAKE_SERVO = 0;
constexpr int PCA_CH_LEFT_BRAKE_SERVO  = 7;
// Generous standard hobby-servo pulse range. If a servo buzzes/strains at
// either extreme without actually reaching the angle you asked for, its
// real range is narrower than this - back off the min/max here rather than
// the requested angle.
constexpr int SERVO_PULSE_MIN_US = 500;   // ~0 degrees
constexpr int SERVO_PULSE_MAX_US = 2500;  // ~180 degrees

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

int angleToTicks(float angleDeg) {
    angleDeg = constrain(angleDeg, 0.0f, 180.0f);
    float pulseUs = SERVO_PULSE_MIN_US +
        (angleDeg / 180.0f) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US);
    float periodUs = 1000000.0f / SERVO_PWM_FREQ_HZ;
    return (int)(pulseUs / periodUs * 4096.0f + 0.5f);
}

void setServoAngle(int channel, float angleDeg) {
    pca.setPWM(channel, 0, angleToTicks(angleDeg));
}

void printHelp() {
    Serial.println();
    Serial.println("Brake servo calibration tool");
    Serial.println("  R<angle>        e.g. R90");
    Serial.println("  L<angle>        e.g. L45");
    Serial.println("  R<angle> L<angle>   both at once, e.g. R120 L60");
    Serial.println("  ?               show this help");
    Serial.println("Move in small steps and watch the physical brake.");
    Serial.println();
}

void handleToken(const String &token) {
    if (token.length() < 2) return;
    char side = token.charAt(0);
    float angle = token.substring(1).toFloat();
    if (side == 'R' || side == 'r') {
        setServoAngle(PCA_CH_RIGHT_BRAKE_SERVO, angle);
        Serial.print("right -> "); Serial.print(angle); Serial.println(" deg");
    } else if (side == 'L' || side == 'l') {
        setServoAngle(PCA_CH_LEFT_BRAKE_SERVO, angle);
        Serial.print("left  -> "); Serial.print(angle); Serial.println(" deg");
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Wire.begin();
    pca.begin();
    pca.setPWMFreq(SERVO_PWM_FREQ_HZ);
    printHelp();
    Serial.println("Idle - both channels undriven until you send a command.");
}

void loop() {
    static String line;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            line.trim();
            if (line == "?") {
                printHelp();
            } else if (line.length() > 0) {
                int spaceIdx = line.indexOf(' ');
                if (spaceIdx == -1) {
                    handleToken(line);
                } else {
                    handleToken(line.substring(0, spaceIdx));
                    handleToken(line.substring(spaceIdx + 1));
                }
            }
            line = "";
        } else {
            line += c;
        }
    }
}
