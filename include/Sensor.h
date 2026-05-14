#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM42670P.h>
#include "Constants.h"
#include "Encoder.h"

// Em 1DOF yaw será sempre 0
struct SensorData {
    float pitch;        // graus — acelerômetro IMU
    float yaw;          // graus — giroscópio integrado
    float encPitchDeg;  // graus — encoder pitch
    float encYawDeg;    // graus — encoder yaw
};

class Sensor {
public:
    void begin();
    void attachEncoders(Encoder* pitch, Encoder* yaw);
    SensorData read();

private:
    ICM42670P imu{Wire, 0};         
    Encoder* encPitch   = nullptr;
    Encoder* encYaw     = nullptr;
    float yawAccum      = 0.0f;
    unsigned long lastReadUs = 0;
};

#endif  // SENSOR_H