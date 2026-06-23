#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), pulseCount(0), offsetDeg(0.0f) {}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(pinA), isr, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinB), isr, this, CHANGE);
}

void IRAM_ATTR Encoder::isr(void* arg) {
    Encoder* enc = static_cast<Encoder*>(arg);
    bool a = digitalRead(enc->pinA);
    bool b = digitalRead(enc->pinB);
    enc->pulseCount += (a == b) ? 1 : -1;
}

float Encoder::getAngleDeg() const {
    float pulsesDeg = (pulseCount / static_cast<float>(ENCODER_PPR_X4)) * 360.0f;
    return pulsesDeg + offsetDeg;
}

void Encoder::reset() {
    pulseCount = 0;
    offsetDeg  = 0.0f;
}

void Encoder::setOffsetDeg(float deg) {
    offsetDeg = deg;
}
