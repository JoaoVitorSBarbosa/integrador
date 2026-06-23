#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "constants.h"

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);
    void begin();

    // Ângulo em graus, considerando o offset persistido
    float getAngleDeg() const;

    // Pulsos brutos desde o último reset()
    int32_t getPulseCount() const { return pulseCount; }

    // Zera os pulsos (define zero mecânico desta sessão)
    void reset();

    // Injeta um offset em graus carregado da flash no boot.
    // getAngleDeg() retorna pulsos→graus + offsetDeg.
    void setOffsetDeg(float deg);
    float getOffsetDeg() const { return offsetDeg; }

private:
    uint8_t  pinA;
    uint8_t  pinB;
    volatile int32_t pulseCount;
    float    offsetDeg = 0.0f;

    static void IRAM_ATTR isr(void* arg);
};

#endif  // ENCODER_H
