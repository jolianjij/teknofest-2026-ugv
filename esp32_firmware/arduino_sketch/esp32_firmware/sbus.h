#pragma once
#include <Arduino.h>

// Minimal SBUS frame reader/decoder (16 channels, 11 bits each).
class SbusReceiver {
public:
    void begin(HardwareSerial &serial, int rxPin, int txPin);
    void update(); // call every loop iteration

    uint16_t channel(int idx) const { return channels_[idx]; }
    bool failsafe() const { return failsafe_; }
    unsigned long lastFrameMs() const { return lastFrameMs_; }

private:
    static constexpr int FRAME_LEN = 25;
    static constexpr uint8_t START_BYTE = 0x0F;

    HardwareSerial *serial_ = nullptr;
    uint8_t buf_[FRAME_LEN] = {0};
    int bufPos_ = 0;
    uint16_t channels_[16] = {0};
    bool failsafe_ = true; // fail safe (disarmed) until a good frame is seen
    unsigned long lastFrameMs_ = 0;

    void decodeFrame();
};
