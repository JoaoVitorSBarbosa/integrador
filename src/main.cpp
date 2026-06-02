#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "constants.h"
#include "env.h"

#include "Encoder.h"
#include "Motor.h"
#include "IMU.h"
#include "MotorCmd.h"
#include "WebManager.h"


static QueueHandle_t ctrlQueue;
static QueueHandle_t telemQueue;
static QueueHandle_t motorCmdQueue;

struct TaskParams {
    QueueHandle_t ctrlQueue;
    QueueHandle_t telemQueue;
    QueueHandle_t motorCmdQueue;
    IMU*          sensor;
    Motor*        motorPitch;
    Motor*        motorYaw;
    WebManager*   web;
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
        xQueueSend(p->telemQueue, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(1000 / FREQ_SENSOR_HZ));
    }
}

// -----------------------------------------------------------------
// Core 0 — Telemetria Serial (CSV) + Web (SSE) a FREQ_TELEMETRY_HZ
// -----------------------------------------------------------------
void taskTelemetry(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    SensorData data;
    uint32_t t = 0;

    xQueueReceive(p->telemQueue, &data, portMAX_DELAY);
    Serial.println("t_ms,pitch_deg,yaw_deg,enc_pitch_deg,enc_yaw_deg");

    for (;;) {
        if (xQueueReceive(p->telemQueue, &data, portMAX_DELAY) == pdPASS) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f\n",
                t, data.pitch, data.yaw,
                data.encPitchDeg, data.encYawDeg);

            if (p->web) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                    "{\"pitch\":%.2f,\"yaw\":%.2f,\"encPitch\":%.2f,\"encYaw\":%.2f}",
                    data.pitch, data.yaw, data.encPitchDeg, data.encYawDeg);
                p->web->sendTelemetry(buf);
            }

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
    MotorCmd   lastCmd;     // default: mode=CONTROL, duty=0

    xQueueReceive(p->ctrlQueue, &last, portMAX_DELAY);

    for (;;) {
        if (xQueueReceive(p->ctrlQueue, &data, 0) == pdPASS) {
            last = data;
        }

        // Atualiza comando se o WebManager postou um novo
        MotorCmd newCmd;
        if (xQueueReceive(p->motorCmdQueue, &newCmd, 0) == pdPASS) {
            lastCmd = newCmd;
        }

        if (lastCmd.mode == MotorCmd::Mode::TEST) {
            p->motorPitch->setVelocidade(lastCmd.dutyPitch);
            p->motorYaw->setVelocidade(lastCmd.dutyYaw);
        } else {
            // TODO (MCM): substituir por PID
            float errPitch = 0.0f - last.pitch;
            p->motorPitch->setVelocidade(errPitch);

            float errYaw = 0.0f - last.yaw;
            p->motorYaw->setVelocidade(errYaw);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / FREQ_CONTROL_HZ));
    }
}

// -----------------------------------------------------------------
// Setup
// -----------------------------------------------------------------
void setup() {
    delay(5000);
    Serial.begin(115200);

    static Encoder encPitch(PIN_ENC_PITCH_A, PIN_ENC_PITCH_B);
    static Encoder encYaw(PIN_ENC_YAW_A,     PIN_ENC_YAW_B);
    encPitch.begin();
    encYaw.begin();

    static IMU sensor;
    sensor.begin();
    sensor.attachEncoders(&encPitch, &encYaw);

    static Motor motorPitch(PIN_MOTOR_PITCH_RPWM, PIN_MOTOR_PITCH_LPWM,
                            LEDC_CH_PITCH_RPWM,   LEDC_CH_PITCH_LPWM);
    static Motor motorYaw(PIN_MOTOR_YAW_RPWM,   PIN_MOTOR_YAW_LPWM,
                          LEDC_CH_YAW_RPWM,     LEDC_CH_YAW_LPWM);
    motorPitch.begin();
    motorYaw.begin();

    ctrlQueue     = xQueueCreate(5,  sizeof(SensorData));
    telemQueue    = xQueueCreate(10, sizeof(SensorData));
    motorCmdQueue = xQueueCreate(1,  sizeof(MotorCmd));   // depth 1: só o comando mais recente importa

    static WebManager webMan(AP_SSID, AP_PASSWORD);
    webMan.attachMotorQueue(motorCmdQueue);
    webMan.begin();

    static TaskParams sensorParams  = {ctrlQueue, telemQueue, nullptr,       &sensor, nullptr,      nullptr,    nullptr};
    static TaskParams controlParams = {ctrlQueue, nullptr,    motorCmdQueue, nullptr, &motorPitch,  &motorYaw,  nullptr};
    static TaskParams telemParams   = {nullptr,   telemQueue, nullptr,       nullptr, nullptr,      nullptr,    &webMan};

    xTaskCreatePinnedToCore(
        taskSensor,    "sensor",    TASK_STACK_SIZE, &sensorParams,  1, NULL, 0);
    xTaskCreatePinnedToCore(
        taskTelemetry, "telemetry", TASK_STACK_SIZE, &telemParams,   1, NULL, 0);
    xTaskCreatePinnedToCore(
        taskControl,   "control",   TASK_STACK_SIZE, &controlParams, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }
