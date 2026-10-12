#pragma once

enum class RobotState {
    BOOT,
    DISARMED,
    MANUAL,
    AUTONOMOUS,
    FAILSAFE,
};

void ledStatusBegin();
void ledStatusUpdate(RobotState state);
