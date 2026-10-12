#pragma once
#include "led_status.h" // RobotState

void oledBegin();
void oledUpdate(RobotState state, float headingDeg, bool sbusOk, bool rosAgentOk);
