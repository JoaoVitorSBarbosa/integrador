#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Constants.h"

class Motor {
public:
    Motor(uint8_t rpwm, uint8_t lpwm, uint8_t ledcChRpwm, uint8_t ledcChLpwm);
    void begin();
    void setVelocidade(float sinalControle);  // range [-255, 255]
    void parar();

private:
    uint8_t pinRPWM;
    uint8_t pinLPWM;
    uint8_t chRPWM;
    uint8_t chLPWM;
};

#endif  // MOTOR_H