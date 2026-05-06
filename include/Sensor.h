#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

struct SensorData {
    float pitch;
    float yaw;
};

class Sensor {
public:
    void begin();
    SensorData read();
};

#endif