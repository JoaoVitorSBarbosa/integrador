#include "Encoder.h"
#include "Constants.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), pulseCount(0) {}

void Encoder::begin() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(pinA), isrHandler, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinB), isrHandler, this, CHANGE);
}

void IRAM_ATTR Encoder::isrHandler(void* arg) {
    Encoder* enc = static_cast<Encoder*>(arg);
    bool a = digitalRead(enc->pinA);
    bool b = digitalRead(enc->pinB);
    enc->pulseCount += (a == b) ? 1 : -1;
}

float Encoder::getAngleDeg() const {
    // 600 PPR x4 (quadratura CHANGE em A e B) = 2400 pulsos/volta
    return (pulseCount / (float)ENCODER_PPR_X4) * 360.0f;
}

void Encoder::reset() {
    pulseCount = 0;
}