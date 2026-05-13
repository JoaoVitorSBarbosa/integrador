#ifndef CONSTANTS_H
#define CONSTANTS_H

// Pinos —  1DOF (pitch axis)
#define PIN_MOTOR_PITCH_PWM  18
#define PIN_MOTOR_PITCH_DIR  19

// --- Motores ---
#define PIN_MOTOR_PITCH_RPWM  18
#define PIN_MOTOR_PITCH_LPWM  19
#define PIN_MOTOR_YAW_RPWM    20
#define PIN_MOTOR_YAW_LPWM    21

// --- Encoders ---
#define PIN_ENC_PITCH_A   34
#define PIN_ENC_PITCH_B   35
#define PIN_ENC_YAW_A     36
#define PIN_ENC_YAW_B     39
#define ENCODER_PPR_X4    2400  // 600PPR x4 (quadratura CHANGE em A e B)

// --- IMU ---
#define IMU_I2C_SDA   8
#define IMU_I2C_SCL   9
#define IMU_I2C_ADDR  0x68  // AP_AD0=0

// --- FreeRTOS ---
#define FREQ_SENSOR_HZ    50
#define FREQ_CONTROL_HZ   100
#define TASK_STACK_SIZE   4096

// --- Controle PID (pitch) ---
#define KP  1.5f
#define KI  0.0f
#define KD  0.0f

#endif