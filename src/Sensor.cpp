#include "Sensor.h"

void Sensor::begin() {
    Wire.begin(IMU_SDA, IMU_SCL);
    int ret = imu.begin();
    if (ret != 0) {
        Serial.printf("[Sensor] Falha ao iniciar IMU, codigo %d\n", ret);
        return;
    }
    imu.startAccel(100, 2);
    imu.startGyro(100, 500);
    lastReadUs = micros();
    Serial.println("[Sensor] IIM-42652 iniciado");
}

void Sensor::attachEncoders(Encoder* pitch, Encoder* yaw) {
    encPitch = pitch;
    encYaw   = yaw;
}

SensorData Sensor::read() {
    SensorData data = {};

    inv_imu_sensor_event_t event;
    if (imu.getDataFromFifo(event) == 0) {
        float ay = event.accel[1] / 1000.0f;   // mg → g
        float az = event.accel[2] / 1000.0f;
        float gz = event.gyro[2]  / 1000.0f;   // mdps → dps

        // Pitch via acelerômetro 
        data.pitch = atan2f(ay, az) * 180.0f / PI;

        // Yaw via integração do giroscópio
        unsigned long now = micros();
        float dt          = (now - lastReadUs) / 1e6f;
        lastReadUs        = now;
        yawAccum         += gz * dt;
        data.yaw          = yawAccum;
    } else {
        lastReadUs = micros();
    }

    // Encoders — yaw retorna 0 se não conectado (modo 1DOF físico)
    data.encPitchDeg = (encPitch != nullptr) ? encPitch->getAngleDeg() : 0.0f;
    data.encYawDeg   = (encYaw   != nullptr) ? encYaw->getAngleDeg()   : 0.0f;

    return data;
}