#pragma once
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// PCA9685 channels 8-15 drive an 8-channel relay board's IN1-IN8 as simple
// on/off outputs, using the PWM chip's full-on/full-off override bits
// (setting bit 12 of the ON or OFF count register) rather than an actual
// PWM duty cycle.
//
// Every call site in loop() calls this unconditionally, every iteration,
// regardless of whether the requested state actually changed - measured as
// the dominant cost keeping the main loop from running fast enough to
// service the gimbal steppers at their configured rate (each is a full I2C
// transaction over Wire). Caching the last state actually written per
// channel and skipping the transaction when nothing changed turns ~8
// unconditional I2C writes/loop into ~0 in steady state, with zero change
// to any caller.
inline void pcaDigitalWrite(Adafruit_PWMServoDriver &pca, uint8_t channel, bool on) {
    static int8_t lastState[16] = { -1, -1, -1, -1, -1, -1, -1, -1,
                                     -1, -1, -1, -1, -1, -1, -1, -1 };
    if (channel >= 16) return;
    int8_t wanted = on ? 1 : 0;
    if (lastState[channel] == wanted) return;
    lastState[channel] = wanted;

    bool driveHigh = on ? !RELAY_ACTIVE_LOW : RELAY_ACTIVE_LOW;
    if (driveHigh) {
        pca.setPWM(channel, 4096, 0); // full on
    } else {
        pca.setPWM(channel, 0, 4096); // full off
    }
}
