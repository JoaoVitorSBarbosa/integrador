#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <LittleFS.h>
#include <esp_system.h>
#include <math.h>

#include "constants.h"
#include "env.h"
#include "Encoder.h"
#include "Motor.h"
#include "IMU.h"
#include "MotorCmd.h"
#include "Calibration.h"
#include "WebManager.h"

// ─── Telemetria estendida ─────────────────────────────────────────────────────
struct TelemData {
    SensorData sensor;
    float uPitch    = 0.0f;   // sinal de controle pitch (antes do clamp)
    float uYaw      = 0.0f;   // sinal de controle yaw
    ControlMode mode = ControlMode::DOF1;
};

// ─── Queues globais ───────────────────────────────────────────────────────────
static QueueHandle_t sensorQueue;      // SensorData:  taskSensor → taskControl
static QueueHandle_t telemQueue;       // TelemData:   taskControl → taskTelemetry
static QueueHandle_t motorCmdQueue;    // MotorCmd:    web → taskControl  (depth 1)
static QueueHandle_t calibCmdQueue;    // CalibCmd:    web → taskCalib    (depth 1)
static QueueHandle_t ctrlParamsQueue;  // ControlParams: web → taskControl (depth 1)

// ─── Parâmetros das tasks ─────────────────────────────────────────────────────
struct TaskParams {
    IMU*          sensor;
    Motor*        motorPitch;
    Motor*        motorYaw;
    Encoder*      encPitch;
    Encoder*      encYaw;
    WebManager*   web;
    CalibData*    calib;
};
static TaskParams* gTp = nullptr;

// ─── Shutdown handler ─────────────────────────────────────────────────────────
static void onShutdown() {
    if (!gTp) return;
    gTp->calib->encPitchDeg = gTp->encPitch->getAngleDeg();
    gTp->calib->encYawDeg   = gTp->encYaw->getAngleDeg();
    Calibration::saveEncoderPositions(*gTp->calib);
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskSensor  (50 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void taskSensor(void* pv) {
    auto* p = static_cast<TaskParams*>(pv);
    for (;;) {
        SensorData d = p->sensor->read();
        xQueueSend(sensorQueue, &d, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000 / FREQ_SENSOR_HZ));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskTelemetry  (50 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void taskTelemetry(void* pv) {
    auto* p = static_cast<TaskParams*>(pv);
    TelemData td;

    // Aguarda primeiro pacote para emitir header correto
    xQueueReceive(telemQueue, &td, portMAX_DELAY);
    Serial.println("t_ms,mode,pitch_deg,yaw_deg,pitchDot,yawDot,"
                   "encPitch_deg,encYaw_deg,uPitch,uYaw");

    uint32_t t = 0;
    for (;;) {
        if (xQueueReceive(telemQueue, &td, portMAX_DELAY) != pdPASS) continue;

        Serial.printf("%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            t, (int)td.mode,
            td.sensor.pitch, td.sensor.yaw,
            td.sensor.gx,    td.sensor.gz,      // θ̇ ≈ gx, Ω̇ ≈ gz (ajustar por eixo)
            td.sensor.encPitchDeg, td.sensor.encYawDeg,
            td.uPitch, td.uYaw);

        if (p->web) {
            char buf[280];
            snprintf(buf, sizeof(buf),
                "{\"mode\":%d,"
                "\"pitch\":%.3f,\"yaw\":%.3f,"
                "\"pitchDot\":%.3f,\"yawDot\":%.3f,"
                "\"encPitch\":%.2f,\"encYaw\":%.2f,"
                "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                "\"uPitch\":%.2f,\"uYaw\":%.2f}",
                (int)td.mode,
                td.sensor.pitch, td.sensor.yaw,
                td.sensor.gx,    td.sensor.gz,
                td.sensor.encPitchDeg, td.sensor.encYawDeg,
                td.sensor.ax, td.sensor.ay, td.sensor.az,
                td.uPitch, td.uYaw);
            p->web->sendTelemetry(buf);
        }
        t += (1000 / FREQ_TELEMETRY_HZ);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 1 — taskControl  (100 Hz)
// Todas as grandezas angulares em radianos / rad·s⁻¹ internamente.
// ─────────────────────────────────────────────────────────────────────────────
void taskControl(void* pv) {
    auto* p = static_cast<TaskParams*>(pv);

    TickType_t   lastWake = xTaskGetTickCount();
    SensorData   last     = {};
    MotorCmd     motorCmd;
    ControlParams cp;       // começa com defaults do construtor (DOF1, tudo zero)

    // Estado do integrador
    float intPitch = 0.0f;   // ∫(rp − θ)dt
    float intYaw   = 0.0f;   // ∫(ry − Ω)dt

    // Derivada numérica calculada só quando chega novo dado (50 Hz) e
    // mantida (ZOH) nos ticks intermediários — evita aliasing 0 / 2×.
    float prevThetaEnc  = 0.0f;
    float prevOmegaEnc  = 0.0f;
    float thetaDot      = 0.0f;
    float omegaDot      = 0.0f;
    unsigned long prevUs       = micros();
    unsigned long prevSensorUs = micros();

    // Aguarda primeiro dado de sensor
    xQueueReceive(sensorQueue, &last, portMAX_DELAY);
    prevThetaEnc = last.encPitchDeg * (PI / 180.0f);
    prevOmegaEnc = last.encYawDeg   * (PI / 180.0f);
    prevUs       = micros();
    prevSensorUs = prevUs;

    for (;;) {
        // Atualiza sensor — derivada recalculada apenas quando há dado novo
        SensorData newSensor;
        if (xQueueReceive(sensorQueue, &newSensor, 0) == pdPASS) {
            float thetaNew       = newSensor.encPitchDeg * (PI / 180.0f);
            float omegaNew       = newSensor.encYawDeg   * (PI / 180.0f);
            unsigned long nowSns = micros();
            float sdT            = (nowSns - prevSensorUs) / 1e6f;
            if (sdT > 0.0f && sdT < 0.5f) {
                thetaDot = (thetaNew - prevThetaEnc) / sdT;
                omegaDot = (omegaNew - prevOmegaEnc) / sdT;
            }
            prevThetaEnc = thetaNew;
            prevOmegaEnc = omegaNew;
            prevSensorUs = nowSns;
            last = newSensor;
        }

        // Atualiza modo de acionamento direto (calibração/teste)
        MotorCmd newMotorCmd;
        if (xQueueReceive(motorCmdQueue, &newMotorCmd, 0) == pdPASS) motorCmd = newMotorCmd;

        // Atualiza parâmetros — sempre reseta integradores ao receber novos params
        ControlParams newCp;
        if (xQueueReceive(ctrlParamsQueue, &newCp, 0) == pdPASS) {
            intPitch = 0.0f;
            intYaw   = 0.0f;
            cp = newCp;
            Serial.printf("[Ctrl] Modo=%d refP=%.3f refY=%.3f\n",
                          (int)cp.mode, cp.refPitch, cp.refYaw);
        }

        // dt para integração (período real do tick de controle)
        unsigned long nowUs = micros();
        float dt = (nowUs - prevUs) / 1e6f;
        prevUs = nowUs;
        if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / FREQ_CONTROL_HZ;

        // Grandezas angulares em rad
        float theta = last.encPitchDeg * (PI / 180.0f);   // θ  — encoder pitch
        float omega  = last.encYawDeg   * (PI / 180.0f);   // Ω  — encoder yaw
        // thetaDot / omegaDot mantidos desde última atualização de sensor (ZOH)

        float uPitch = 0.0f, uYaw = 0.0f;

        // ── Modo CALIBRATION — taskCalibration tem controle exclusivo ────────
        if (motorCmd.mode == MotorCmd::Mode::CALIBRATION) {
            uPitch = 0.0f; uYaw = 0.0f;
            goto send_telem;
        }

        // ── Modo TEST (teste manual via web) ──────────────────────────────
        if (motorCmd.mode == MotorCmd::Mode::TEST) {
            uPitch = motorCmd.dutyPitch;
            uYaw   = motorCmd.dutyYaw;
            p->motorPitch->setVelocidade(uPitch);
            p->motorYaw->setVelocidade(uYaw);
            goto send_telem;
        }

        switch (cp.mode) {

            // ── MALHA ABERTA ──────────────────────────────────────────────
            case ControlMode::OPEN_LOOP:
                uPitch = cp.openLoopPitch;
                uYaw   = cp.openLoopYaw;
                intPitch = 0.0f;
                intYaw   = 0.0f;
                break;

            // ── 1-DOF ─────────────────────────────────────────────────────
            // U = Ki1·∫(rp − θ)dt − [Kx1 Kx2]·[θ, θ̇]ᵀ
            // Vy = 0
            case ControlMode::DOF1:
                intPitch += (cp.refPitch - theta) * dt;
                uPitch = cp.Ki1 * intPitch
                         - (cp.Kx1 * theta + cp.Kx2 * thetaDot);
                uYaw   = 0.0f;
                intYaw = 0.0f;
                break;

            // ── 2-DOF ─────────────────────────────────────────────────────
            // [Vp]   [Ki1  0 ]   [∫(πp−θ)]   [Kx1 Kx2 Kx3 Kx4]   [θ ]
            // [  ] = [       ] · [       ] − [                  ] · [Ω ]
            // [Vy]   [0  Ki2 ]   [∫(πy−Ω)]   [Kx5 Kx6 Kx7 Kx8]   [θ̇]
            //                                                        [Ω̇]
            case ControlMode::DOF2:
                intPitch += (cp.refPitch - theta) * dt;
                intYaw   += (cp.refYaw   - omega)  * dt;
                uPitch = cp.Ki1 * intPitch
                         - (cp.Kx1*theta + cp.Kx2*omega + cp.Kx3*thetaDot + cp.Kx4*omegaDot);
                uYaw   = cp.Ki2 * intYaw
                         - (cp.Kx5*theta + cp.Kx6*omega + cp.Kx7*thetaDot + cp.Kx8*omegaDot);
                break;
        }

        // Clamp e aplica
        p->motorPitch->setVelocidade(
            constrain(uPitch, -(float)MOTOR_PWM_MAX_DUTY, (float)MOTOR_PWM_MAX_DUTY));
        p->motorYaw->setVelocidade(
            constrain(uYaw,   -(float)MOTOR_PWM_MAX_DUTY, (float)MOTOR_PWM_MAX_DUTY));

        send_telem: {
            TelemData td;
            td.sensor = last;
            td.uPitch = uPitch;
            td.uYaw   = uYaw;
            td.mode   = cp.mode;
            xQueueSend(telemQueue, &td, 0);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / FREQ_CONTROL_HZ));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Core 0 — taskCalibration
// ─────────────────────────────────────────────────────────────────────────────
void taskCalibration(void* pv) {
    auto* p = static_cast<TaskParams*>(pv);
    CalibCmd cmd;

    auto sendStatus = [&](const char* msg, bool done, bool err = false) {
        if (!p->web) return;
        char buf[140];
        snprintf(buf, sizeof(buf),
            "{\"msg\":\"%s\",\"done\":%s,\"error\":%s}",
            msg, done ? "true":"false", err ? "true":"false");
        p->web->sendCalibStatus(buf);
        Serial.printf("[Calib] %s\n", msg);
    };

    for (;;) {
        xQueueReceive(calibCmdQueue, &cmd, portMAX_DELAY);

        // Para calibração de motores: CALIBRATION (taskControl sai do caminho).
        // Para demais tipos: TEST duty=0 (motores ativamente parados).
        {
            MotorCmd pause;
            pause.mode = (cmd.type == CalibCmd::Type::MOTORS)
                         ? MotorCmd::Mode::CALIBRATION
                         : MotorCmd::Mode::TEST;
            xQueueOverwrite(motorCmdQueue, &pause);
        }
        vTaskDelay(pdMS_TO_TICKS(200));

        switch (cmd.type) {
            case CalibCmd::Type::RESET:
                LittleFS.remove("/calibration.json");
                *p->calib = CalibData{};
                p->sensor->setCalibration(0,0,0,0,0,0);
                p->sensor->resetYaw();
                p->motorPitch->setDeadband(0,0);
                p->motorYaw->setDeadband(0,0);
                p->encPitch->reset(); p->encYaw->reset();
                sendStatus("Calibracao resetada.", true);
                break;

            case CalibCmd::Type::ENCODERS:
                sendStatus("Zerando encoders...", false);
                Calibration::zeroEncoders(*p->encPitch, *p->encYaw, *p->calib);
                sendStatus("Encoders zerados. Posicao salva no flash.", true);
                break;

            case CalibCmd::Type::IMU:
                sendStatus("Calibrando IMU... mantenha sensor parado.", false);
                Calibration::calibrateIMU(*p->sensor, *p->calib);
                p->sensor->setCalibration(
                    p->calib->imuOffsetAx, p->calib->imuOffsetAy, p->calib->imuOffsetAz,
                    p->calib->imuOffsetGx, p->calib->imuOffsetGy, p->calib->imuOffsetGz);
                p->sensor->resetYaw();
                Calibration::save(*p->calib);
                sendStatus("IMU calibrado.", true);
                break;

            case CalibCmd::Type::MOTORS:
                sendStatus("Calibrando motores... braco livre.", false);
                Calibration::calibrateMotors(
                    *p->motorPitch, *p->motorYaw,
                    *p->encPitch,   *p->encYaw, *p->calib);
                p->motorPitch->setDeadband(
                    p->calib->motorPitchDeadbandFwd, p->calib->motorPitchDeadbandRev);
                p->motorYaw->setDeadband(
                    p->calib->motorYawDeadbandFwd,   p->calib->motorYawDeadbandRev);
                Calibration::save(*p->calib);
                sendStatus("Motores calibrados.", true);
                break;

            default: break;
        }

        MotorCmd resume; resume.mode = MotorCmd::Mode::CONTROL;
        xQueueOverwrite(motorCmdQueue, &resume);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("[FW] v-OTA-test");

    if (!LittleFS.begin(true)) Serial.println("[Setup] ERRO: LittleFS falhou.");

    static Encoder encPitch(PIN_ENC_PITCH_A, PIN_ENC_PITCH_B);
    static Encoder encYaw  (PIN_ENC_YAW_A,   PIN_ENC_YAW_B);
    encPitch.begin(); encYaw.begin();

    static IMU sensor;
    sensor.begin();
    sensor.attachEncoders(&encPitch, &encYaw);

    static Motor motorPitch(PIN_MOTOR_PITCH_RPWM, PIN_MOTOR_PITCH_LPWM,
                            LEDC_CH_PITCH_RPWM,   LEDC_CH_PITCH_LPWM);
    static Motor motorYaw  (PIN_MOTOR_YAW_RPWM,   PIN_MOTOR_YAW_LPWM,
                            LEDC_CH_YAW_RPWM,     LEDC_CH_YAW_LPWM);
    motorPitch.begin(); motorYaw.begin();

    static CalibData calib = Calibration::load();
    sensor.setCalibration(
        calib.imuOffsetAx, calib.imuOffsetAy, calib.imuOffsetAz,
        calib.imuOffsetGx, calib.imuOffsetGy, calib.imuOffsetGz);
    motorPitch.setDeadband(calib.motorPitchDeadbandFwd, calib.motorPitchDeadbandRev);
    motorYaw.setDeadband  (calib.motorYawDeadbandFwd,   calib.motorYawDeadbandRev);

    if (calib.encPitchDeg != 0.0f || calib.encYawDeg != 0.0f) {
        encPitch.setOffsetDeg(calib.encPitchDeg);
        encYaw.setOffsetDeg(calib.encYawDeg);
        Serial.printf("[Setup] Encoders restaurados: pitch=%.2f° yaw=%.2f°\n",
                      calib.encPitchDeg, calib.encYawDeg);
    }

    sensorQueue     = xQueueCreate(5,  sizeof(SensorData));
    telemQueue      = xQueueCreate(10, sizeof(TelemData));
    motorCmdQueue   = xQueueCreate(1,  sizeof(MotorCmd));
    calibCmdQueue   = xQueueCreate(1,  sizeof(CalibCmd));
    ctrlParamsQueue = xQueueCreate(1,  sizeof(ControlParams));

    static WebManager webMan(AP_SSID, AP_PASSWORD);
    webMan.attachMotorQueue(motorCmdQueue);
    webMan.attachCalibQueue(calibCmdQueue);
    webMan.attachCtrlParamsQueue(ctrlParamsQueue);
    webMan.begin();

    static TaskParams tp = {
        &sensor, &motorPitch, &motorYaw, &encPitch, &encYaw, &webMan, &calib
    };
    gTp = &tp;
    esp_register_shutdown_handler(onShutdown);
    // Verifica conteudo real do LittleFS apos OTA
    File f = LittleFS.open("/index.html", "r");
    if (f) {
        String line = f.readStringUntil('\n');  // primeira linha
        f.close();
        Serial.printf("[FS] index.html linha 1: %s\n", line.c_str());
    } else {
        Serial.println("[FS] Falha ao abrir index.html");
    }
    
    xTaskCreatePinnedToCore(taskSensor,      "sensor",    TASK_STACK_SIZE,     &tp, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskTelemetry,   "telemetry", TASK_STACK_SIZE,     &tp, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskControl,     "control",   TASK_STACK_SIZE,     &tp, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskCalibration, "calib",     TASK_STACK_SIZE * 2, &tp, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
