#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "Constants.h"

class Motor {
  private:
    uint8_t pinPWM;
    uint8_t pinDIR;
  
  public:
    Motor(uint8_t pwm, uint8_t dir);
    void begin();
    void setVelocidade(float sinalControle);
};
#endif