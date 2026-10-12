#include "oled_display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 display(128, 64, &Wire, -1);

void oledBegin() {
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.display();
}

static const char *stateName(RobotState s) {
    switch (s) {
        case RobotState::BOOT:       return "BOOT";
        case RobotState::DISARMED:   return "DISARMED";
        case RobotState::MANUAL:     return "MANUAL";
        case RobotState::AUTONOMOUS: return "AUTONOMOUS";
        case RobotState::FAILSAFE:   return "FAILSAFE";
        case RobotState::ESTOP:      return "E-STOP";
    }
    return "?";
}

void oledUpdate(RobotState state, float headingDeg, bool sbusOk, bool rosAgentOk) {
    static unsigned long lastDraw = 0;
    if (millis() - lastDraw < 200) return; // ~5 Hz refresh, keep the I2C bus free for BNO055/PCA9685
    lastDraw = millis();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("State: ");   display.println(stateName(state));
    display.print("Heading: "); display.print(headingDeg, 1); display.println(" deg");
    display.print("SBUS: ");    display.println(sbusOk ? "OK" : "LOST");
    display.print("Agent: ");   display.println(rosAgentOk ? "OK" : "---");
    display.display();
}
