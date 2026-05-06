#include "Sensor.h"

void Sensor::begin() {
    // TODO: Wire.begin() + init IIM-42652 via I2C
}

SensorData Sensor::read() {
    // TODO: substituir por leitura real do IMU
    return { .pitch = 15.5f, .yaw = 0.0f };
}