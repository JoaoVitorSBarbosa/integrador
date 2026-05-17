#ifndef CONSTANTS_H
#define CONSTANTS_H

// --- Motor Pitch ---
#define PIN_MOTOR_PITCH_RPWM    17
#define PIN_MOTOR_PITCH_LPWM    18

// --- Motor Yaw ---
#define PIN_MOTOR_YAW_RPWM      15
#define PIN_MOTOR_YAW_LPWM      16

// --- Encoder Pitch ---
#define PIN_ENC_PITCH_A         4
#define PIN_ENC_PITCH_B         5

// --- Encoder Yaw ---
#define PIN_ENC_YAW_A           6
#define PIN_ENC_YAW_B           7

// 600PPR x4 (CHANGE em A e B) = 2400 pulsos/volta
#define ENCODER_PPR_X4          2400

// --- IMU ---
#define IMU_SDA                 8
#define IMU_SCL                 9
#define IMU_I2C_ADDR            0x68    // AP_AD0 = 0

// --- LEDC PWM ---
#define MOTOR_PWM_FREQ_HZ       20000
#define MOTOR_PWM_RESOLUTION    8       // 8 bits = duty 0-255
#define MOTOR_PWM_MAX_DUTY      255
#define LEDC_CH_PITCH_RPWM      0
#define LEDC_CH_PITCH_LPWM      1
#define LEDC_CH_YAW_RPWM        2
#define LEDC_CH_YAW_LPWM        3

// --- FreeRTOS ---
// Core 0: sensores + telemetria
// Core 1: malha de controle
#define FREQ_SENSOR_HZ      50
#define FREQ_CONTROL_HZ     100
#define FREQ_TELEMETRY_HZ   50
#define TASK_STACK_SIZE     4096

// --- Ganhos PID  ---
#define KP  1.5f
#define KI  0.0f
#define KD  0.0f

#define FREQ_SENSOR_HZ    50
#define FREQ_CONTROL_HZ  100
#define TASK_STACK_SIZE  4096
constexpr float KP = 1.5f;

#endif  // CONSTANTS_H
