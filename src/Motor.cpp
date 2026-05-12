#include "Motor.h"

Motor::Motor(uint8_t pwm, uint8_t dir)
    : pinPWM(pwm), pinDIR(dir) {}

void Motor::begin() {
    pinMode(pinDIR, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    digitalWrite(pinDIR, LOW);
    analogWrite(pinPWM, 0);
}

void Motor::setVelocidade(float sinalControle) {
    int duty = (int)constrain(abs(sinalControle), 0.0f, 255.0f);
    digitalWrite(pinDIR, sinalControle >= 0 ? HIGH : LOW);
    // TODO: substituir por ledcWrite quando hardware disponivel
    Serial.printf("[Motor] dir=%s duty=%d\n",
        sinalControle >= 0 ? "FWD" : "REV", duty);
}
