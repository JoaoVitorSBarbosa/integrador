#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "Constants.h"
#include "Motor.h"
#include "Sensor.h"

// Estrutura para passar parâmetros para as tasks
struct TaskParams {
    QueueHandle_t queue;
    Sensor* sensor;
    Motor* motor;
};

// ==========================================
// CORE 0: LEITURA DE SENSORES (TELEMETRIA)
// ==========================================
void loopLeituraSensores(void *pvParameters) {
  TaskParams* params = (TaskParams*) pvParameters;
  Sensor* sensor = params->sensor;
  QueueHandle_t queue = params->queue;

  DadosSensores dadosAtuais;
  for (;;) {
  
    dadosAtuais = sensor->lerDados();
    
    xQueueSend(queue, &dadosAtuais, portMAX_DELAY);
    
    Serial.printf("Core 0 (Sensores): Pitch lido: %.2f\n", dadosAtuais.anguloPitch);
    vTaskDelay(pdMS_TO_TICKS(1000 / FREQ_LEITURA_SENSORES_HZ)); 
  }
}

// ==========================================
// CORE 1: MALHA DE CONTROLE
// ==========================================
void loopControle(void *pvParameters) {
  TaskParams* params = (TaskParams*) pvParameters;
  Motor* motor = params->motor;
  QueueHandle_t queue = params->queue;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FREQ_CONTROLE_HZ); 
  
  DadosSensores dadosRecebidos;
  for (;;) {
    
    if (xQueueReceive(queue, &dadosRecebidos, 0) == pdPASS) {
      
      
      float erro = 0.0 - dadosRecebidos.anguloPitch;
      
      float acao_controle = erro * 1.5; // Exemplo de PID
      
      motor->setVelocidade(acao_controle);

      Serial.printf("Core 1 (Controle): Erro: %.2f, Ação: %.2f\n", erro, acao_controle);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

// ==========================================
// SETUP INICIAL
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Inicia hardware
  static Motor motorPitch(PIN_MOTOR_PITCH_PWM, PIN_MOTOR_PITCH_DIR);
  motorPitch.begin();

  static Sensor sensorPitch;
  sensorPitch.begin();

  // Cria a fila - 5 leituras
  QueueHandle_t filaSensores = xQueueCreate(5, sizeof(DadosSensores));

  // Prepara os parâmetros 
  static TaskParams taskParamsCore0;
  taskParamsCore0.queue = filaSensores;
  taskParamsCore0.sensor = &sensorPitch;
  taskParamsCore0.motor = nullptr; // Não usado nesta task

  static TaskParams taskParamsCore1;
  taskParamsCore1.queue = filaSensores;
  taskParamsCore1.sensor = nullptr; // Não usado nesta task
  taskParamsCore1.motor = &motorPitch;

  // Inicia o RTOS Dual-Core
  // Core 0: Leitura de Sensores (Telemetria)
  xTaskCreatePinnedToCore(loopLeituraSensores, "LeituraSensores", TASK_STACK_SIZE, &taskParamsCore0, 1, NULL, 0);
  // Core 1: Malha de Controle
  xTaskCreatePinnedToCore(loopControle, "Controle", TASK_STACK_SIZE, &taskParamsCore1, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }
