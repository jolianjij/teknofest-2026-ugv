#include "motor_mixer.h"
#include "config.h"
#include <algorithm>
#include <cmath>

MotorOutputs mixDifferential(float linear, float angular) {
    MotorOutputs out;
    out.left  = linear - angular;
    out.right = linear + angular;
    float maxMag = std::max(1.0f, std::max(std::fabs(out.left), std::fabs(out.right)));
    out.left  /= maxMag;
    out.right /= maxMag;
    return out;
}

int normalizedToPulseUs(float value) {
    value = std::max(-1.0f, std::min(1.0f, value));
    if (value >= 0.0f) {
        return ESC_PULSE_NEUTRAL_US + (int)(value * (ESC_PULSE_MAX_US - ESC_PULSE_NEUTRAL_US));
    }
    return ESC_PULSE_NEUTRAL_US + (int)(value * (ESC_PULSE_NEUTRAL_US - ESC_PULSE_MIN_US));
}
