#include "IMU.h"
#include <math.h>

IMU::IMU() {}

void IMU::begin() {
    Serial.println("[IMU] Iniciando Wire...");
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(10);
    delay(50);

    Serial.printf("[IMU] Tentando MPU-6050 em 0x%02X...\n", IMU_I2C_ADDR);

    if (!imu.begin(IMU_I2C_ADDR, &Wire)) {
        Serial.println("[IMU] FALHA: MPU-6050 nao respondeu.");
        Serial.println("[IMU]   AD0 em GND  -> 0x68");
        Serial.println("[IMU]   AD0 em 3V3  -> 0x69");
        return;
    }

    imu.setAccelerometerRange(MPU6050_RANGE_2_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // Leitura de sanidade
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    Serial.printf("[IMU] Leitura inicial: accel=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f)\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z,
                  g.gyro.x, g.gyro.y, g.gyro.z);

    lastReadUs = micros();
    available  = true;
    Serial.println("[IMU] MPU-6050 iniciado.");
}

void IMU::setCalibration(float ax, float ay, float az,
                          float gx, float gy, float gz) {
    offAx = ax; offAy = ay; offAz = az;
    offGx = gx; offGy = gy; offGz = gz;
    Serial.printf("[IMU] Calibracao aplicada: offAy=%.4f offAz=%.4f offGz=%.4f\n",
                  offAy, offAz, offGz);
}

void IMU::attachEncoders(Encoder* pitch, Encoder* yaw) {
    encPitch = pitch;
    encYaw   = yaw;
}

SensorData IMU::readInternal(bool applyOffset) {
    SensorData data = {};

    data.encPitchDeg = encPitch ? encPitch->getAngleDeg() : 0.0f;
    data.encYawDeg   = encYaw   ? encYaw->getAngleDeg()   : 0.0f;

    if (!available) return data;

    sensors_event_t accelEvt, gyroEvt, tempEvt;
    imu.getEvent(&accelEvt, &gyroEvt, &tempEvt);

    // Acelerômetro: m/s² → g
    data.ax = accelEvt.acceleration.x / 9.80665f;
    data.ay = accelEvt.acceleration.y / 9.80665f;
    data.az = accelEvt.acceleration.z / 9.80665f;

    // Giroscópio: rad/s → °/s
    data.gx = gyroEvt.gyro.x * (180.0f / PI);
    data.gy = gyroEvt.gyro.y * (180.0f / PI);
    data.gz = gyroEvt.gyro.z * (180.0f / PI);

    if (applyOffset) {
        data.ax -= offAx;
        data.ay -= offAy;
        data.az -= offAz;
        data.gx -= offGx;
        data.gy -= offGy;
        data.gz -= offGz;
    }

    data.pitch = atan2f(data.ay, data.az) * 180.0f / PI;

    unsigned long now = micros();
    float dt = (now - lastReadUs) / 1000000.0f;
    lastReadUs = now;

    if (dt > 0.0f && dt < 0.5f) {
        yawAccum += data.gz * dt;
    }
    data.yaw = yawAccum;

    return data;
}

SensorData IMU::read()    { return readInternal(true);  }
SensorData IMU::readRaw() { return readInternal(false); }
