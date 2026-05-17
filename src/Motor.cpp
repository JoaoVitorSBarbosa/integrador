#include "Motor.h"

Motor::Motor(uint8_t rpwm, uint8_t lpwm, uint8_t ledcChRpwm, uint8_t ledcChLpwm)
    : pinRPWM(rpwm), pinLPWM(lpwm), chRPWM(ledcChRpwm), chLPWM(ledcChLpwm) {}

void Motor::begin() {
    ledcSetup(chRPWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
    ledcSetup(chLPWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(pinRPWM, chRPWM);
    ledcAttachPin(pinLPWM, chLPWM);
    parar();
}

void Motor::setVelocidade(float sinalControle) {
    int duty = static_cast<int>(
        constrain(abs(sinalControle), 0.0f, static_cast<float>(MOTOR_PWM_MAX_DUTY)));
    if (sinalControle > 0.0f) {
        ledcWrite(chRPWM, duty);
        ledcWrite(chLPWM, 0);
    } else if (sinalControle < 0.0f) {
        ledcWrite(chRPWM, 0);
        ledcWrite(chLPWM, duty);
    } else {
        parar();
    }
}

void Motor::parar() {
    ledcWrite(chRPWM, 0);
    ledcWrite(chLPWM, 0);
}
