#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include <Arduino.h>

struct MotorCmd {
    enum class Mode : uint8_t { CONTROL, TEST } mode = Mode::CONTROL;
    float dutyPitch = 0.0f;
    float dutyYaw   = 0.0f;
};

#endif
