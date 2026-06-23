#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <LittleFS.h>
#include <esp_system.h>   // esp_register_shutdown_handler

#include "constants.h"
#include "env.h"
#include "Encoder.h"
#include "Motor.h"
#include "IMU.h"
#include "MotorCmd.h"
#include "Calibration.h"
#include "WebManager.h"

// ─── Queues globais ───────────────────────────────────────────────────────────
static QueueHandle_t ctrlQueue;
static QueueHandle_t telemQueue;
static QueueHandle_t motorCmdQueue;
static QueueHandle_t calibCmdQueue;

// ─── Parâmetros das tasks ─────────────────────────────────────────────────────
struct TaskParams {
    QueueHandle_t ctrlQueue;
    QueueHandle_t telemQueue;
    QueueHandle_t motorCmdQueue;
    QueueHandle_t calibCmdQueue;
    IMU*          sensor;
    Motor*        motorPitch;
    Motor*        motorYaw;
    Encoder*      encPitch;
    Encoder*      encYaw;
    WebManager*   web;
    CalibData*    calib;
};

// Ponteiro global acessível pelo shutdown handler (função C, sem contexto)
static TaskParams* gTp = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Shutdown handler — chamado pelo ESP32 antes de desligar/reiniciar
//
// Persiste a posição atual dos encoders no LittleFS para que o próximo
// boot os restaure via setOffsetDeg().
// ─────────────────────────────────────────────────────────────────────────────
static void onShutdown() {
    if (!gTp) return;
    gTp->calib->encPitchDeg = gTp->encPitch->getAngleDeg();
    gTp->calib->encYawDeg   = gTp->encYaw->getAngleDeg();
    Calibration::saveEncoderPositions(*gTp->calib);
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskSensor
// ─────────────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskTelemetry
// ─────────────────────────────────────────────────────────────────────────────
void taskTelemetry(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    SensorData data;
    uint32_t t = 0;

    xQueueReceive(p->telemQueue, &data, portMAX_DELAY);
    Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,yaw_deg,enc_pitch_deg,enc_yaw_deg");

    for (;;) {
        if (xQueueReceive(p->telemQueue, &data, portMAX_DELAY) == pdPASS) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                data.ax, data.ay, data.az,
                data.gx, data.gy, data.gz,
                data.pitch, data.yaw,
                data.encPitchDeg, data.encYawDeg);

            if (p->web) {
                char buf[220];
                snprintf(buf, sizeof(buf),
                    "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                    "\"pitch\":%.2f,\"yaw\":%.2f,"
                    "\"encPitch\":%.2f,\"encYaw\":%.2f}",
                    data.ax, data.ay, data.az,
                    data.gx, data.gy, data.gz,
                    data.pitch, data.yaw,
                    data.encPitchDeg, data.encYawDeg);
                p->web->sendTelemetry(buf);
            }
            t += (1000 / FREQ_TELEMETRY_HZ);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 1 — taskControl
// ─────────────────────────────────────────────────────────────────────────────
void taskControl(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    TickType_t lastWake = xTaskGetTickCount();
    SensorData last = {};
    SensorData data;
    MotorCmd   lastCmd;

    xQueueReceive(p->ctrlQueue, &last, portMAX_DELAY);

    for (;;) {
        if (xQueueReceive(p->ctrlQueue, &data, 0) == pdPASS) last = data;

        MotorCmd newCmd;
        if (xQueueReceive(p->motorCmdQueue, &newCmd, 0) == pdPASS) lastCmd = newCmd;

        if (lastCmd.mode == MotorCmd::Mode::TEST) {
            p->motorPitch->setVelocidade(lastCmd.dutyPitch);
            p->motorYaw->setVelocidade(lastCmd.dutyYaw);
        } else {
            // TODO: substituir por LQI
            p->motorPitch->setVelocidade(0.0f - last.pitch);
            p->motorYaw->setVelocidade(0.0f - last.yaw);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / FREQ_CONTROL_HZ));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskCalibration
// ─────────────────────────────────────────────────────────────────────────────
void taskCalibration(void* pvParams) {
    auto* p = static_cast<TaskParams*>(pvParams);
    CalibCmd cmd;

    auto sendStatus = [&](const char* msg, bool done, bool error = false) {
        if (!p->web) return;
        char buf[128];
        snprintf(buf, sizeof(buf),
            "{\"msg\":\"%s\",\"done\":%s,\"error\":%s}",
            msg, done ? "true" : "false", error ? "true" : "false");
        p->web->sendCalibStatus(buf);
        Serial.printf("[Calib] %s\n", msg);
    };

    for (;;) {
        xQueueReceive(p->calibCmdQueue, &cmd, portMAX_DELAY);

        // Para motores antes de qualquer rotina
        MotorCmd pause; pause.mode = MotorCmd::Mode::TEST;
        xQueueOverwrite(p->motorCmdQueue, &pause);
        vTaskDelay(pdMS_TO_TICKS(200));

        switch (cmd.type) {

            case CalibCmd::Type::RESET:
                LittleFS.remove("/calibration.json");
                *p->calib = CalibData{};
                p->sensor->setCalibration(0, 0, 0, 0, 0, 0);
                p->motorPitch->setDeadband(0, 0);
                p->motorYaw->setDeadband(0, 0);
                p->encPitch->reset();
                p->encYaw->reset();
                sendStatus("Calibracao resetada.", true);
                break;

            case CalibCmd::Type::ENCODERS:
                sendStatus("Zerando encoders...", false);
                Calibration::zeroEncoders(*p->encPitch, *p->encYaw, *p->calib);
                sendStatus("Encoders zerados. Posicao salva no flash.", true);
                break;

            case CalibCmd::Type::IMU:
                sendStatus("Calibrando IMU... mantenha o sensor parado.", false);
                Calibration::calibrateIMU(*p->sensor, *p->calib);
                p->sensor->setCalibration(
                    p->calib->imuOffsetAx, p->calib->imuOffsetAy, p->calib->imuOffsetAz,
                    p->calib->imuOffsetGx, p->calib->imuOffsetGy, p->calib->imuOffsetGz);
                Calibration::save(*p->calib);
                sendStatus("IMU calibrado.", true);
                break;

            case CalibCmd::Type::MOTORS:
                sendStatus("Calibrando motores... braco deve estar livre.", false);
                Calibration::calibrateMotors(
                    *p->motorPitch, *p->motorYaw,
                    *p->encPitch,   *p->encYaw,
                    *p->calib);
                p->motorPitch->setDeadband(
                    p->calib->motorPitchDeadbandFwd,
                    p->calib->motorPitchDeadbandRev);
                p->motorYaw->setDeadband(
                    p->calib->motorYawDeadbandFwd,
                    p->calib->motorYawDeadbandRev);
                Calibration::save(*p->calib);
                sendStatus("Motores calibrados.", true);
                break;

            default: break;
        }

        // Retorna ao controle
        MotorCmd resume; resume.mode = MotorCmd::Mode::CONTROL;
        xQueueOverwrite(p->motorCmdQueue, &resume);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    delay(3000);
    Serial.begin(115200);

    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERRO: LittleFS falhou.");
    }

    static Encoder encPitch(PIN_ENC_PITCH_A, PIN_ENC_PITCH_B);
    static Encoder encYaw  (PIN_ENC_YAW_A,   PIN_ENC_YAW_B);
    encPitch.begin();
    encYaw.begin();

    static IMU sensor;
    sensor.begin();
    sensor.attachEncoders(&encPitch, &encYaw);

    static Motor motorPitch(PIN_MOTOR_PITCH_RPWM, PIN_MOTOR_PITCH_LPWM,
                            LEDC_CH_PITCH_RPWM,   LEDC_CH_PITCH_LPWM);
    static Motor motorYaw  (PIN_MOTOR_YAW_RPWM,   PIN_MOTOR_YAW_LPWM,
                            LEDC_CH_YAW_RPWM,     LEDC_CH_YAW_LPWM);
    motorPitch.begin();
    motorYaw.begin();

    // Carrega calibração persistida
    static CalibData calib = Calibration::load();

    // Aplica offsets do IMU
    sensor.setCalibration(
        calib.imuOffsetAx, calib.imuOffsetAy, calib.imuOffsetAz,
        calib.imuOffsetGx, calib.imuOffsetGy, calib.imuOffsetGz);

    // Aplica dead zones dos motores
    motorPitch.setDeadband(calib.motorPitchDeadbandFwd, calib.motorPitchDeadbandRev);
    motorYaw.setDeadband  (calib.motorYawDeadbandFwd,   calib.motorYawDeadbandRev);

    // Restaura posição dos encoders do último desligamento controlado
    // Se o drone foi movido com energia desligada, o operador deve
    // usar "Zerar Encoders" na interface web para corrigir.
    if (calib.encPitchDeg != 0.0f || calib.encYawDeg != 0.0f) {
        encPitch.setOffsetDeg(calib.encPitchDeg);
        encYaw.setOffsetDeg(calib.encYawDeg);
        Serial.printf("[Setup] Posicao dos encoders restaurada: pitch=%.2f° yaw=%.2f°\n",
                      calib.encPitchDeg, calib.encYawDeg);
    }

    // Queues
    ctrlQueue      = xQueueCreate(5,  sizeof(SensorData));
    telemQueue     = xQueueCreate(10, sizeof(SensorData));
    motorCmdQueue  = xQueueCreate(1,  sizeof(MotorCmd));
    calibCmdQueue  = xQueueCreate(1,  sizeof(CalibCmd));

    static WebManager webMan(AP_SSID, AP_PASSWORD);
    webMan.attachMotorQueue(motorCmdQueue);
    webMan.attachCalibQueue(calibCmdQueue);
    webMan.begin();

    static TaskParams tp = {
        ctrlQueue, telemQueue, motorCmdQueue, calibCmdQueue,
        &sensor, &motorPitch, &motorYaw, &encPitch, &encYaw,
        &webMan, &calib
    };
    gTp = &tp;

    // Registra o shutdown handler do ESP-IDF — chamado em esp_restart()
    esp_register_shutdown_handler(onShutdown);

    xTaskCreatePinnedToCore(taskSensor,      "sensor",    TASK_STACK_SIZE,     &tp, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskTelemetry,   "telemetry", TASK_STACK_SIZE,     &tp, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskControl,     "control",   TASK_STACK_SIZE,     &tp, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskCalibration, "calib",     TASK_STACK_SIZE * 2, &tp, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
