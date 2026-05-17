#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "constants.h"
#include "env.h"

#include "Encoder.h"
#include "Motor.h"
#include "Sensor.h"
#include "WebManager.h"


static QueueHandle_t ctrlQueue;
static QueueHandle_t telemQueue;

struct TaskParams {
    QueueHandle_t ctrlQueue;
    QueueHandle_t telemQueue;
    Sensor*       sensor;
    Motor*        motorPitch;
    Motor*        motorYaw;
};

// -----------------------------------------------------------------
// Core 0 — Leitura de sensores a FREQ_SENSOR_HZ
// -----------------------------------------------------------------
void taskSensor(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    SensorData data;
    for (;;) {
        data = p->sensor->read();
        xQueueSend(p->ctrlQueue,  &data, portMAX_DELAY);
        xQueueSend(p->telemQueue, &data, 0);    // não bloqueia
        vTaskDelay(pdMS_TO_TICKS(1000 / FREQ_SENSOR_HZ));
    }
}

// -----------------------------------------------------------------
// Core 0 — Telemetria CSV para MATLAB/Simulink a FREQ_TELEMETRY_HZ
// -----------------------------------------------------------------
void taskTelemetry(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    SensorData data;
    uint32_t t = 0;

    Serial.println("t_ms,pitch_deg,yaw_deg,enc_pitch_deg,enc_yaw_deg");

    for (;;) {
        if (xQueueReceive(p->telemQueue, &data, portMAX_DELAY) == pdPASS) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f\n",
                t, data.pitch, data.yaw,
                data.encPitchDeg, data.encYawDeg);
            t += (1000 / FREQ_TELEMETRY_HZ);
        }
    }
}

// -----------------------------------------------------------------
// Core 1 — Malha de controle a FREQ_CONTROL_HZ
// -----------------------------------------------------------------
void taskControl(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    TickType_t lastWake = xTaskGetTickCount();
    SensorData last     = {};
    SensorData data;
    // first read: block until sensor delivers
    xQueueReceive(p->queue, &last, portMAX_DELAY);

    for (;;) {
        if (xQueueReceive(p->ctrlQueue, &data, 0) == pdPASS) {
            last = data;
        }

        // --- Eixo Pitch ---
        float errPitch = 0.0f - last.pitch;
        // TODO (MCM): substituir por PID
        p->motorPitch->setVelocidade(errPitch * KP);

        // --- Eixo Yaw ---
        // Em 1DOF: yaw travado fisicamente → last.yaw = 0 → errYaw = 0 → duty = 0
        float errYaw = 0.0f - last.yaw;
        // TODO (MCM): substituir por PID
        p->motorYaw->setVelocidade(errYaw * KP);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / FREQ_CONTROL_HZ));
    }
}

// -----------------------------------------------------------------
// Setup
// -----------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    static WebManager webMan(AP_SSID, AP_PASSWORD);
    webMan.begin();
    delay(10);

    // Encoders
    static Encoder encPitch(PIN_ENC_PITCH_A, PIN_ENC_PITCH_B);
    static Encoder encYaw(PIN_ENC_YAW_A,     PIN_ENC_YAW_B);
    encPitch.begin();
    encYaw.begin();
    delay(10);

    // Sensor (IMU + encoders)
    static Sensor sensor;
    sensor.begin();
    sensor.attachEncoders(&encPitch, &encYaw);

    // Motores
    static Motor motorPitch(PIN_MOTOR_PITCH_RPWM, PIN_MOTOR_PITCH_LPWM,
                            LEDC_CH_PITCH_RPWM,   LEDC_CH_PITCH_LPWM);
    static Motor motorYaw(PIN_MOTOR_YAW_RPWM,   PIN_MOTOR_YAW_LPWM,
                          LEDC_CH_YAW_RPWM,     LEDC_CH_YAW_LPWM);
    motorPitch.begin();
    motorYaw.begin();
    delay(10);

    static QueueHandle_t queue = xQueueCreate(5, sizeof(SensorData));
    assert(queue != NULL);

    // Filas
    ctrlQueue  = xQueueCreate(5,  sizeof(SensorData));
    telemQueue = xQueueCreate(10, sizeof(SensorData));

    // Parâmetros das tasks
    static TaskParams sensorParams  = {ctrlQueue, telemQueue, &sensor,  nullptr,     nullptr};
    static TaskParams controlParams = {ctrlQueue, nullptr,    nullptr,  &motorPitch, &motorYaw};
    static TaskParams telemParams   = {nullptr,   telemQueue, nullptr,  nullptr,     nullptr};

    // Core 0: sensores + telemetria
    xTaskCreatePinnedToCore(
        taskSensor,    "sensor",    TASK_STACK_SIZE, &sensorParams,  1, NULL, 0);
    xTaskCreatePinnedToCore(
        taskTelemetry, "telemetry", TASK_STACK_SIZE, &telemParams,   1, NULL, 0);

    // Core 1: controle crítico
    xTaskCreatePinnedToCore(
        taskControl,   "control",   TASK_STACK_SIZE, &controlParams, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }