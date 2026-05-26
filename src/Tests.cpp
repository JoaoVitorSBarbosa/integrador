#include "Tests.h"

Tests::Tests() {}
Tests::~Tests() {}

void Tests::motor(Motor& obj) {
    Serial.println("[Test::motor] Rampando sentido horario...");
    for (uint16_t i = 0; i <= 255; i++) {
        obj.setVelocidade(i);
        delay(10);
    }
    delay(500);

    Serial.println("[Test::motor] Rampando sentido anti-horario...");
    for (uint16_t i = 0; i <= 255; i++) {
        obj.setVelocidade(-static_cast<int>(i));
        delay(10);
    }
    delay(500);

    obj.setVelocidade(0);
    Serial.println("[Test::motor] Concluido.");
}

void Tests::encoder(Encoder& obj) {
    obj.reset();
    Serial.println("[Test::encoder] Lendo angulo por 5s (gire o eixo)...");
    for (uint8_t i = 0; i < 50; i++) {
        Serial.printf("  %.2f deg\n", obj.getAngleDeg());
        delay(100);
    }
    Serial.println("[Test::encoder] Concluido.");
}

void Tests::imu(IMU& obj) {
    Serial.println("[Test::imu] Lendo dados por 5s...");
    for (uint8_t i = 0; i < 50; i++) {
        SensorData d = obj.read();
        Serial.printf("  pitch: %6.2f deg  yaw: %6.2f deg  encPitch: %6.2f deg  encYaw: %6.2f deg\n",
            d.pitch, d.yaw, d.encPitchDeg, d.encYawDeg);
        delay(100);
    }
    Serial.println("[Test::imu] Concluido.");
}

template<> void Tests::singleTest<Motor>(Motor& obj)     { motor(obj); }
template<> void Tests::singleTest<Encoder>(Encoder& obj) { encoder(obj); }
template<> void Tests::singleTest<IMU>(IMU& obj)         { imu(obj); }
