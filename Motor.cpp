#include "Motor.h"

Motor::Motor(uint8_t pwm, uint8_t dir) {
    this->pinPWM = pwm;
    this->pinDIR = dir;
}

void Motor::begin() {
    pinMode(pinDIR, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    // Configurações do PWM do ESP32 virão aqui...
}

void Motor::setVelocidade(float sinalControle) {
    // Lógica para transformar o float do PID em PWM real
    
}