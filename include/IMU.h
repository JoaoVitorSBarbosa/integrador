#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM42670P.h>
#include "constants.h"
#include "Encoder.h"

// Em 1DOF yaw será sempre 0
struct SensorData {
    float pitch;        // graus — IMU acelerômetro
    float yaw;          // graus — integração gyro
    float encPitchDeg;  // encoder pitch
    float encYawDeg;    // encoder yaw
};

class IMU {
public:
    IMU();

    void begin();
    void attachEncoders(Encoder* pitch, Encoder* yaw);
    SensorData read();

private:
    ICM42670 imu;

    Encoder* encPitch = nullptr;
    Encoder* encYaw   = nullptr;

    bool available = false;
    float yawAccum = 0.0f;
    unsigned long lastReadUs = 0;
};

#endif
