#include "Sensor.h"
#include <math.h>

Sensor::Sensor() : imu(Wire, 0) {}

void Sensor::begin() {
    Wire.begin(IMU_SDA, IMU_SCL);

    int ret = imu.begin();

    if (ret != 0) {
        Serial.printf("[Sensor] Falha ao iniciar IMU, codigo %d\n", ret);
        while (true) {
            delay(1000);
        }
    }

    imu.startAccel(100, 2);
    imu.startGyro(100, 500);

    lastReadUs = micros();

    Serial.println("[Sensor] ICM42670 iniciado");
}

void Sensor::attachEncoders(Encoder* pitch, Encoder* yaw) {
    encPitch = pitch;
    encYaw = yaw;
}

SensorData Sensor::read() {
    SensorData data = {};

    inv_imu_sensor_event_t event;

    if (imu.getDataFromRegisters(event) == 0) {
        float ay = event.accel[1] / 1000.0f;
        float az = event.accel[2] / 1000.0f;
        float gz = event.gyro[2] / 1000.0f;

        // Pitch usando acelerômetro
        data.pitch = atan2f(ay, az) * 180.0f / PI;

        // Integra yaw via giroscópio
        unsigned long now = micros();
        float dt = (now - lastReadUs) / 1000000.0f;
        lastReadUs = now;

        yawAccum += gz * dt;
        data.yaw = yawAccum;
    }

    // Encoders
    data.encPitchDeg = (encPitch != nullptr)
        ? encPitch->getAngleDeg()
        : 0.0f;

    data.encYawDeg = (encYaw != nullptr)
        ? encYaw->getAngleDeg()
        : 0.0f;

    return data;
}