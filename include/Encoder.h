#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();
    float getAngleDeg() const;
    void reset();

private:
    uint8_t pinA;
    uint8_t pinB;
    volatile int32_t pulseCount;

    static void IRAM_ATTR isrHandler(void* arg);
};

#endif