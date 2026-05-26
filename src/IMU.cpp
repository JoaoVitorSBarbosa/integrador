#include "IMU.h"
#include <math.h>

IMU::IMU() : imu(Wire, 0) {}

void IMU::begin() {
    Wire.begin(IMU_SDA, IMU_SCL);

    int ret = imu.begin();
    if (ret != 0) {
        Serial.printf("[IMU] Falha ao iniciar ICM42670, codigo %d\n", ret);
        while (true) {
            delay(1000);
        }
    }

    imu.startAccel(100, 2);
    imu.startGyro(100, 500);

    lastReadUs = micros();

    Serial.println("[IMU] ICM42670 iniciado");
}

void IMU::attachEncoders(Encoder* pitch, Encoder* yaw) {
    encPitch = pitch;
    encYaw   = yaw;
}

SensorData IMU::read() {
    SensorData data = {};

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
