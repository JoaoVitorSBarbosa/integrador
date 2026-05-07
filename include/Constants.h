#ifndef CONSTANTS_H
#define CONSTANTS_H

// Pinos —  1DOF (pitch axis)
#define PIN_MOTOR_PITCH_PWM  18
#define PIN_MOTOR_PITCH_DIR  19

// Frequencias das tasks FreeRTOS
#define FREQ_SENSOR_HZ    50
#define FREQ_CONTROL_HZ  100
#define TASK_STACK_SIZE  4096
constexpr float KP = 1.5f;

#endif
