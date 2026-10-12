#pragma once

enum class RobotState {
    BOOT,
    DISARMED,
    MANUAL,
    AUTONOMOUS,
    FAILSAFE,
    ESTOP,
};

void ledStatusBegin();
void ledStatusUpdate(RobotState state);
