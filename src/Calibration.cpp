#include "Calibration.h"

static const char* CALIB_PATH = "/calibration.json";

// ─────────────────────────────────────────────────────────────────────────────
// load / save
// ─────────────────────────────────────────────────────────────────────────────

CalibData Calibration::load() {
    CalibData d;
    File f = LittleFS.open(CALIB_PATH, "r");
    if (!f) {
        Serial.println("[Calib] Arquivo nao encontrado — usando defaults.");
        return d;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.printf("[Calib] Erro ao ler JSON: %s\n", err.c_str());
        return d;
    }

    d.imuOffsetAx = doc["imuOffsetAx"] | 0.0f;
    d.imuOffsetAy = doc["imuOffsetAy"] | 0.0f;
    d.imuOffsetAz = doc["imuOffsetAz"] | 0.0f;
    d.imuOffsetGx = doc["imuOffsetGx"] | 0.0f;
    d.imuOffsetGy = doc["imuOffsetGy"] | 0.0f;
    d.imuOffsetGz = doc["imuOffsetGz"] | 0.0f;

    d.motorPitchDeadbandFwd = doc["motorPitchDeadbandFwd"] | 0.0f;
    d.motorPitchDeadbandRev = doc["motorPitchDeadbandRev"] | 0.0f;
    d.motorYawDeadbandFwd   = doc["motorYawDeadbandFwd"]   | 0.0f;
    d.motorYawDeadbandRev   = doc["motorYawDeadbandRev"]   | 0.0f;

    d.encPitchDeg = doc["encPitchDeg"] | 0.0f;
    d.encYawDeg   = doc["encYawDeg"]   | 0.0f;

    d.imuCalibrated   = doc["imuCalibrated"]   | false;
    d.motorCalibrated = doc["motorCalibrated"] | false;

    Serial.printf("[Calib] Carregado — IMU(%s) Motor(%s) Enc(pitch=%.2f° yaw=%.2f°)\n",
        d.imuCalibrated   ? "ok" : "nao",
        d.motorCalibrated ? "ok" : "nao",
        d.encPitchDeg, d.encYawDeg);
    return d;
}

void Calibration::save(const CalibData& d) {
    JsonDocument doc;
    doc["imuOffsetAx"] = d.imuOffsetAx;
    doc["imuOffsetAy"] = d.imuOffsetAy;
    doc["imuOffsetAz"] = d.imuOffsetAz;
    doc["imuOffsetGx"] = d.imuOffsetGx;
    doc["imuOffsetGy"] = d.imuOffsetGy;
    doc["imuOffsetGz"] = d.imuOffsetGz;

    doc["motorPitchDeadbandFwd"] = d.motorPitchDeadbandFwd;
    doc["motorPitchDeadbandRev"] = d.motorPitchDeadbandRev;
    doc["motorYawDeadbandFwd"]   = d.motorYawDeadbandFwd;
    doc["motorYawDeadbandRev"]   = d.motorYawDeadbandRev;

    doc["encPitchDeg"] = d.encPitchDeg;
    doc["encYawDeg"]   = d.encYawDeg;

    doc["imuCalibrated"]   = d.imuCalibrated;
    doc["motorCalibrated"] = d.motorCalibrated;
    // encodersZeroed não é persistido intencionalmente

    File f = LittleFS.open(CALIB_PATH, "w");
    if (!f) { Serial.println("[Calib] Erro ao abrir arquivo para escrita."); return; }
    serializeJson(doc, f);
    f.close();
    Serial.println("[Calib] Salvo em /calibration.json");
}

void Calibration::saveEncoderPositions(const CalibData& d) {
    // Lê o JSON atual, atualiza só os campos de encoder e regrava.
    // Evita sobrescrever campos não relacionados com valores antigos.
    JsonDocument doc;
    File fr = LittleFS.open(CALIB_PATH, "r");
    if (fr) { deserializeJson(doc, fr); fr.close(); }

    doc["encPitchDeg"] = d.encPitchDeg;
    doc["encYawDeg"]   = d.encYawDeg;

    File fw = LittleFS.open(CALIB_PATH, "w");
    if (!fw) { Serial.println("[Calib] Erro ao salvar posicao dos encoders."); return; }
    serializeJson(doc, fw);
    fw.close();
    Serial.printf("[Calib] Posicao salva — pitch=%.2f° yaw=%.2f°\n",
                  d.encPitchDeg, d.encYawDeg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibração do IMU
// ─────────────────────────────────────────────────────────────────────────────

void Calibration::calibrateIMU(IMU& imu, CalibData& out,
                                int samples, int periodMs) {
    Serial.printf("[Calib] IMU: coletando %d amostras (%d ms)...\n",
                  samples, samples * periodMs);

    double sumAx = 0, sumAy = 0, sumAz = 0;
    double sumGx = 0, sumGy = 0, sumGz = 0;

    for (int i = 0; i < samples; i++) {
        SensorData d = imu.readRaw();
        sumAx += d.ax;  sumAy += d.ay;  sumAz += d.az;
        sumGx += d.gx;  sumGy += d.gy;  sumGz += d.gz;
        delay(periodMs);
    }

    out.imuOffsetAx = (float)(sumAx / samples);
    out.imuOffsetAy = (float)(sumAy / samples);
    out.imuOffsetAz = (float)(sumAz / samples);
    out.imuOffsetGx = (float)(sumGx / samples);
    out.imuOffsetGy = (float)(sumGy / samples);
    out.imuOffsetGz = (float)(sumGz / samples);
    out.imuCalibrated = true;

    Serial.printf("[Calib] IMU offset: ax=%.4f ay=%.4f az=%.4f | gx=%.4f gy=%.4f gz=%.4f\n",
                  out.imuOffsetAx, out.imuOffsetAy, out.imuOffsetAz,
                  out.imuOffsetGx, out.imuOffsetGy, out.imuOffsetGz);
}

// ─────────────────────────────────────────────────────────────────────────────
// Calibração de dead zone dos motores
// ─────────────────────────────────────────────────────────────────────────────

static float findDeadband(Motor& motor, Encoder& enc,
                          bool forward, int stepDuty, int stepMs, int timeoutMs) {
    enc.reset();
    delay(50);

    int32_t startPulse = enc.getPulseCount();
    int duty = 0;
    unsigned long tStart = millis();

    while (duty <= MOTOR_PWM_MAX_DUTY) {
        if ((int)(millis() - tStart) > timeoutMs) {
            Serial.printf("[Calib]   Timeout sem movimento (dir=%s)\n", forward ? "FWD" : "REV");
            motor.parar();
            return 0.0f;
        }
        motor.setVelocidadeRaw(forward ? (float)duty : -(float)duty);
        delay(stepMs);

        if (abs(enc.getPulseCount() - startPulse) >= 3) {
            motor.parar();
            Serial.printf("[Calib]   Deadband %s = %d duty\n", forward ? "FWD" : "REV", duty);
            return (float)duty;
        }
        duty += stepDuty;
    }

    motor.parar();
    Serial.printf("[Calib]   Motor nao moveu (dir=%s)\n", forward ? "FWD" : "REV");
    return 0.0f;
}

void Calibration::calibrateMotors(Motor& pitch, Motor& yaw,
                                   Encoder& encPitch, Encoder& encYaw,
                                   CalibData& out,
                                   int stepDuty, int stepMs, int timeoutMs) {
    Serial.println("[Calib] Motor Pitch FWD...");
    out.motorPitchDeadbandFwd = findDeadband(pitch, encPitch, true,  stepDuty, stepMs, timeoutMs);
    Serial.println("[Calib] Motor Pitch REV...");
    out.motorPitchDeadbandRev = findDeadband(pitch, encPitch, false, stepDuty, stepMs, timeoutMs);
    Serial.println("[Calib] Motor Yaw FWD...");
    out.motorYawDeadbandFwd   = findDeadband(yaw, encYaw, true,  stepDuty, stepMs, timeoutMs);
    Serial.println("[Calib] Motor Yaw REV...");
    out.motorYawDeadbandRev   = findDeadband(yaw, encYaw, false, stepDuty, stepMs, timeoutMs);
    out.motorCalibrated = true;
    Serial.println("[Calib] Calibracao de motores concluida.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Zeragem dos encoders
// ─────────────────────────────────────────────────────────────────────────────

void Calibration::zeroEncoders(Encoder& encPitch, Encoder& encYaw,
                                CalibData& out) {
    encPitch.reset();   // zera pulseCount e offsetDeg
    encYaw.reset();
    out.encPitchDeg   = 0.0f;
    out.encYawDeg     = 0.0f;
    out.encodersZeroed = true;
    // Salva o zero no JSON para que o próximo boot carregue 0 como posição
    saveEncoderPositions(out);
    Serial.println("[Calib] Encoders zerados e 0 deg salvo no flash.");
}
