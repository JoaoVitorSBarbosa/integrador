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

void Motor::setDeadband(float fwd, float rev) {
    deadbandFwd = fwd;
    deadbandRev = rev;
}

void Motor::applyDuty(float sinal) {
    int duty = static_cast<int>(
        constrain(fabsf(sinal), 0.0f, static_cast<float>(MOTOR_PWM_MAX_DUTY)));
    if (sinal > 0.0f) {
        ledcWrite(chRPWM, duty);
        ledcWrite(chLPWM, 0);
    } else if (sinal < 0.0f) {
        ledcWrite(chRPWM, 0);
        ledcWrite(chLPWM, duty);
    } else {
        parar();
    }
}

void Motor::setVelocidadeRaw(float sinalControle) {
    applyDuty(sinalControle);
}

void Motor::setVelocidade(float sinalControle) {
    if (sinalControle == 0.0f) { parar(); return; }

    // Aplica compensação de dead zone: empurra o sinal para além do limiar
    // de atrito estático, preservando a proporcionalidade acima dele.
    float db = (sinalControle > 0.0f) ? deadbandFwd : deadbandRev;
    float sign = (sinalControle > 0.0f) ? 1.0f : -1.0f;
    float mag  = fabsf(sinalControle);

    // Mapeia [0, MAX] -> [deadband, MAX] linearmente
    float compensated = db + (mag / MOTOR_PWM_MAX_DUTY) * (MOTOR_PWM_MAX_DUTY - db);
    applyDuty(sign * compensated);
}

void Motor::parar() {
    ledcWrite(chRPWM, 0);
    ledcWrite(chLPWM, 0);
}
