#include "IMU.h"
#include <math.h>

IMU::IMU() : imu(Wire, 0) {}

void IMU::begin() {
    // Verifica o estado do barramento antes de inicializar Wire
    // Evita travar se o IMU estiver ausente ou sem alimentacao
    pinMode(IMU_SDA, INPUT_PULLUP);
    pinMode(IMU_SCL, INPUT_PULLUP);
    delayMicroseconds(200);
    if (!digitalRead(IMU_SDA) || !digitalRead(IMU_SCL)) {
        Serial.println("[IMU] Barramento I2C em nivel baixo, continuando sem IMU");
        return;
    }

    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(10);
    delay(10);  // ICM42670 needs ~10ms after VDD before I2C is ready

    // Probe rapido: so chama imu.begin() se o dispositivo responder
    Wire.beginTransmission(IMU_I2C_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[IMU] ICM42670 nao encontrado, continuando sem IMU");
        return;
    }

    if (imu.begin() != 0) {
        Serial.println("[IMU] Falha ao iniciar ICM42670, continuando sem IMU");
        return;
    }

    imu.startAccel(100, 2);
    imu.startGyro(100, 500);
    lastReadUs = micros();
    available = true;
    Serial.println("[IMU] ICM42670 iniciado");
}

void IMU::attachEncoders(Encoder* pitch, Encoder* yaw) {
    encPitch = pitch;
    encYaw   = yaw;
}

SensorData IMU::read() {
    SensorData data = {};

    if (!available) {
        data.encPitchDeg = encPitch ? encPitch->getAngleDeg() : 0.0f;
        data.encYawDeg   = encYaw   ? encYaw->getAngleDeg()   : 0.0f;
        return data;
    }

    inv_imu_sensor_event_t event;

    if (imu.getDataFromRegisters(event) == 0) {
        float ay = event.accel[1] / 1000.0f;
        float az = event.accel[2] / 1000.0f;
        float gz = event.gyro[2] / 1000.0f;

        data.pitch = atan2f(ay, az) * 180.0f / PI;

        unsigned long now = micros();
        float dt = (now - lastReadUs) / 1000000.0f;
        lastReadUs = now;

        yawAccum += gz * dt;
        data.yaw = yawAccum;
    }

    data.encPitchDeg = encPitch ? encPitch->getAngleDeg() : 0.0f;
    data.encYawDeg   = encYaw   ? encYaw->getAngleDeg()   : 0.0f;

    return data;
}
