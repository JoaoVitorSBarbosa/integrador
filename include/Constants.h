#ifndef CONSTANTS_H
#define CONSTANTS_H

// Pinos dos Motores
#define PIN_MOTOR_PITCH_PWM  18
#define PIN_MOTOR_PITCH_DIR  19

// Configurações do RTOS
#define FREQ_LEITURA_SENSORES_HZ 50   // Frequência de leitura dos sensores (Core 0)
#define FREQ_CONTROLE_HZ         100  // Frequência da malha de controle (Core 1)
#define TASK_STACK_SIZE          4096 // Tamanho da stack para as tasks do FreeRTOS

#endif // CONSTANTS_H
