#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "constants.h"

class Motor {
public:
    Motor(uint8_t rpwm, uint8_t lpwm, uint8_t ledcChRpwm, uint8_t ledcChLpwm);
    void begin();

    // Aplica sinal de controle com compensação de deadband.
    // range [-MOTOR_PWM_MAX_DUTY, +MOTOR_PWM_MAX_DUTY]
    void setVelocidade(float sinalControle);

    // Versão sem deadband — usada internamente pela calibração.
    void setVelocidadeRaw(float sinalControle);

    void parar();

    void setDeadband(float fwd, float rev);

private:
    uint8_t pinRPWM;
    uint8_t pinLPWM;
    uint8_t chRPWM;
    uint8_t chLPWM;
    float deadbandFwd = 0.0f;
    float deadbandRev = 0.0f;

    void applyDuty(float sinal);
};

#endif  // MOTOR_H
