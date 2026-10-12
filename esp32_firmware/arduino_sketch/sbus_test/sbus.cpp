#include "sbus.h"

void SbusReceiver::begin(HardwareSerial &serial, int rxPin, int txPin) {
    serial_ = &serial;
    // SBUS: 100000 baud, 8 data bits, even parity, 2 stop bits, inverted signal.
    serial_->begin(100000, SERIAL_8E2, rxPin, txPin, true /*invert*/);
}

void SbusReceiver::update() {
    if (!serial_) return;
    while (serial_->available()) {
        uint8_t b = serial_->read();
        if (bufPos_ == 0 && b != START_BYTE) {
            continue; // resync: wait for a start byte
        }
        buf_[bufPos_++] = b;
        if (bufPos_ == FRAME_LEN) {
            decodeFrame();
            bufPos_ = 0;
        }
    }
    if (millis() - lastFrameMs_ > 500) {
        failsafe_ = true; // no frames at all recently
    }
}

void SbusReceiver::decodeFrame() {
    channels_[0]  = (uint16_t)((buf_[1]      | buf_[2]  << 8)                  & 0x07FF);
    channels_[1]  = (uint16_t)((buf_[2] >> 3 | buf_[3]  << 5)                  & 0x07FF);
    channels_[2]  = (uint16_t)((buf_[3] >> 6 | buf_[4]  << 2 | buf_[5] << 10)  & 0x07FF);
    channels_[3]  = (uint16_t)((buf_[5] >> 1 | buf_[6]  << 7)                  & 0x07FF);
    channels_[4]  = (uint16_t)((buf_[6] >> 4 | buf_[7]  << 4)                  & 0x07FF);
    channels_[5]  = (uint16_t)((buf_[7] >> 7 | buf_[8]  << 1 | buf_[9] << 9)   & 0x07FF);
    channels_[6]  = (uint16_t)((buf_[9] >> 2 | buf_[10] << 6)                  & 0x07FF);
    channels_[7]  = (uint16_t)((buf_[10] >> 5 | buf_[11] << 3)                 & 0x07FF);
    channels_[8]  = (uint16_t)((buf_[12]      | buf_[13] << 8)                 & 0x07FF);
    channels_[9]  = (uint16_t)((buf_[13] >> 3 | buf_[14] << 5)                 & 0x07FF);
    channels_[10] = (uint16_t)((buf_[14] >> 6 | buf_[15] << 2 | buf_[16] << 10)& 0x07FF);
    channels_[11] = (uint16_t)((buf_[16] >> 1 | buf_[17] << 7)                 & 0x07FF);
    channels_[12] = (uint16_t)((buf_[17] >> 4 | buf_[18] << 4)                 & 0x07FF);
    channels_[13] = (uint16_t)((buf_[18] >> 7 | buf_[19] << 1 | buf_[20] << 9) & 0x07FF);
    channels_[14] = (uint16_t)((buf_[20] >> 2 | buf_[21] << 6)                 & 0x07FF);
    channels_[15] = (uint16_t)((buf_[21] >> 5 | buf_[22] << 3)                 & 0x07FF);

    bool frameLost   = buf_[23] & 0x04;
    bool failsafeBit = buf_[23] & 0x08;
    failsafe_ = frameLost || failsafeBit;
    lastFrameMs_ = millis();
}
