#pragma once

struct MotorOutputs {
    float left;  // -1.0 .. 1.0
    float right; // -1.0 .. 1.0
};

// Differential-drive mixing: linear/angular given in normalized -1..1 units.
MotorOutputs mixDifferential(float linear, float angular);

// Convert a normalized -1..1 motor value to an ESC pulse width in microseconds.
int normalizedToPulseUs(float value);
