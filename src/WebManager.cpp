#include "WebManager.h"
#include "constants.h"

WebManager::WebManager(const char* apSsid, const char* apPassword)
    : server(80), events("/api/events") {
    this->ssid     = apSsid;
    this->password = apPassword;
}

void WebManager::attachMotorQueue(QueueHandle_t queue) { motorCmdQueue = queue; }
void WebManager::attachCalibQueue(QueueHandle_t queue) { calibCmdQueue = queue; }

void WebManager::sendTelemetry(const char* json) {
    events.send(json, "telemetry", millis());
}

void WebManager::sendCalibStatus(const char* json) {
    events.send(json, "calib", millis());
}

void WebManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("Erro ao montar o LittleFS!");
        return;
    }
    Serial.println("LittleFS montado.");

    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // ── Páginas ──────────────────────────────────────────────────────────
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/index.html", "text/html");
    });

    // ── LQI params ───────────────────────────────────────────────────────
    server.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/parameters.json", "application/json");
    });

    server.on("/api/params", HTTP_POST, [](AsyncWebServerRequest* req) {
        auto get = [&](const char* key) -> String {
            return req->hasParam(key, true) ? req->getParam(key, true)->value() : "0";
        };
        String json =
            "{\n"
            "    \"k1\": "       + get("k1")       + ",\n"
            "    \"k2\": "       + get("k2")       + ",\n"
            "    \"k3\": "       + get("k3")       + ",\n"
            "    \"k4\": "       + get("k4")       + ",\n"
            "    \"pitch_sp\": " + get("pitch_sp") + ",\n"
            "    \"yaw_sp\": "   + get("yaw_sp")   + "\n"
            "}";
        File f = LittleFS.open("/parameters.json", "w");
        if (!f) { req->send(500, "text/plain", "Erro ao abrir arquivo"); return; }
        f.print(json);
        f.close();
        req->send(200, "text/plain", "OK");
    });

    // ── Calibração: GET status ────────────────────────────────────────────
    server.on("/api/calibration", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (LittleFS.exists("/calibration.json")) {
            req->send(LittleFS, "/calibration.json", "application/json");
        } else {
            req->send(200, "application/json", "{\"imuCalibrated\":false,\"motorCalibrated\":false}");
        }
    });

    // ── Calibração: disparar rotinas via POST ─────────────────────────────
    server.on("/api/calibrate/imu", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (calibCmdQueue) {
            CalibCmd cmd; cmd.type = CalibCmd::Type::IMU;
            xQueueOverwrite(calibCmdQueue, &cmd);
            req->send(200, "text/plain", "IMU calibration started");
        } else {
            req->send(503, "text/plain", "Calib queue not initialized");
        }
    });

    server.on("/api/calibrate/motors", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (calibCmdQueue) {
            CalibCmd cmd; cmd.type = CalibCmd::Type::MOTORS;
            xQueueOverwrite(calibCmdQueue, &cmd);
            req->send(200, "text/plain", "Motor calibration started");
        } else {
            req->send(503, "text/plain", "Calib queue not initialized");
        }
    });

    server.on("/api/calibrate/encoders", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (calibCmdQueue) {
            CalibCmd cmd; cmd.type = CalibCmd::Type::ENCODERS;
            xQueueOverwrite(calibCmdQueue, &cmd);
            req->send(200, "text/plain", "Encoder zeroing started");
        } else {
            req->send(503, "text/plain", "Calib queue not initialized");
        }
    });

    server.on("/api/calibrate/reset", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (calibCmdQueue) {
            CalibCmd cmd; cmd.type = CalibCmd::Type::RESET;
            xQueueOverwrite(calibCmdQueue, &cmd);
            req->send(200, "text/plain", "Calibration reset");
        } else {
            req->send(503, "text/plain", "Calib queue not initialized");
        }
    });

    // ── Teste de motor ────────────────────────────────────────────────────
    server.on("/api/test/motor", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!motorCmdQueue) { req->send(503, "text/plain", "Queue not ready"); return; }
        int duty  = req->hasParam("duty",  true) ? req->getParam("duty",  true)->value().toInt() : 0;
        int motor = req->hasParam("motor", true) ? req->getParam("motor", true)->value().toInt() : 0;
        duty = constrain(duty, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
        MotorCmd cmd;
        cmd.mode      = MotorCmd::Mode::TEST;
        cmd.dutyPitch = (motor == 0) ? (float)duty : 0.0f;
        cmd.dutyYaw   = (motor == 1) ? (float)duty : 0.0f;
        xQueueOverwrite(motorCmdQueue, &cmd);
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/test/motors", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!motorCmdQueue) { req->send(503, "text/plain", "Queue not ready"); return; }
        int dp = req->hasParam("dutyPitch", true) ? req->getParam("dutyPitch", true)->value().toInt() : 0;
        int dy = req->hasParam("dutyYaw",   true) ? req->getParam("dutyYaw",   true)->value().toInt() : 0;
        dp = constrain(dp, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
        dy = constrain(dy, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
        MotorCmd cmd;
        cmd.mode      = MotorCmd::Mode::TEST;
        cmd.dutyPitch = (float)dp;
        cmd.dutyYaw   = (float)dy;
        xQueueOverwrite(motorCmdQueue, &cmd);
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/test/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (motorCmdQueue) { MotorCmd cmd; cmd.mode = MotorCmd::Mode::TEST; xQueueOverwrite(motorCmdQueue, &cmd); }
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/test/control", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (motorCmdQueue) { MotorCmd cmd; cmd.mode = MotorCmd::Mode::CONTROL; xQueueOverwrite(motorCmdQueue, &cmd); }
        req->send(200, "text/plain", "OK");
    });

    // ── SSE ──────────────────────────────────────────────────────────────
    events.onConnect([](AsyncEventSourceClient* client) {
        client->send("{}", "telemetry", millis(), 1000);
    });
    server.addHandler(&events);

    server.begin();
    Serial.println("Web server iniciado na porta 80.");
}
