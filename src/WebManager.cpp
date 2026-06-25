#include "WebManager.h"
#include "constants.h"
#include <ArduinoJson.h>
#include <ElegantOTA.h>

WebManager::WebManager(const char* apSsid, const char* apPassword)
    : server(80), events("/api/events") {
    ssid     = apSsid;
    password = apPassword;
}

void WebManager::attachMotorQueue(QueueHandle_t q)    { motorCmdQueue   = q; }
void WebManager::attachCalibQueue(QueueHandle_t q)    { calibCmdQueue   = q; }
void WebManager::attachCtrlParamsQueue(QueueHandle_t q){ ctrlParamsQueue = q; }
void WebManager::sendTelemetry(const char* json)       { events.send(json, "telemetry", millis()); }
void WebManager::sendCalibStatus(const char* json)     { events.send(json, "calib",     millis()); }

// Helper: lê campo float do body do POST (form-urlencoded)
static float pf(AsyncWebServerRequest* req, const char* key, float def = 0.0f) {
    return req->hasParam(key, true) ? req->getParam(key, true)->value().toFloat() : def;
}
static int pi(AsyncWebServerRequest* req, const char* key, int def = 0) {
    return req->hasParam(key, true) ? req->getParam(key, true)->value().toInt() : def;
}

void WebManager::begin() {
    if (!LittleFS.begin(true)) { Serial.println("LittleFS falhou!"); return; }
    Serial.println("LittleFS montado.");

    WiFi.softAP(ssid, password);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    // ── Página principal ────────────────────────────────────────────────
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/index.html", "text/html");
    });

    // ── GET parâmetros de controle ──────────────────────────────────────
    server.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (LittleFS.exists("/parameters.json"))
            req->send(LittleFS, "/parameters.json", "application/json");
        else
            req->send(200, "application/json", "{\"mode\":1}");
    });

    // ── POST parâmetros de controle ─────────────────────────────────────
    // Recebe todos os campos do formulário, salva JSON e envia para taskControl.
    // Referências chegam em graus e são convertidas para rad aqui.
    server.on("/api/params", HTTP_POST, [this](AsyncWebServerRequest* req) {
        ControlParams cp;
        cp.mode = static_cast<ControlMode>(pi(req, "mode", 1));

        // Referências: web envia graus → controlador usa rad
        cp.refPitch = pf(req, "refPitch") * (PI / 180.0f);
        cp.refYaw   = pf(req, "refYaw")   * (PI / 180.0f);

        cp.Ki1 = pf(req, "Ki1");
        cp.Ki2 = pf(req, "Ki2");

        cp.Kx1 = pf(req, "Kx1"); cp.Kx2 = pf(req, "Kx2");
        cp.Kx3 = pf(req, "Kx3"); cp.Kx4 = pf(req, "Kx4");
        cp.Kx5 = pf(req, "Kx5"); cp.Kx6 = pf(req, "Kx6");
        cp.Kx7 = pf(req, "Kx7"); cp.Kx8 = pf(req, "Kx8");

        cp.openLoopPitch = pf(req, "openLoopPitch");
        cp.openLoopYaw   = pf(req, "openLoopYaw");

        // Persiste — salva os valores originais em graus para exibir de volta
        JsonDocument doc;
        doc["mode"]     = (int)cp.mode;
        doc["refPitch"] = pf(req, "refPitch");   // graus (valor original)
        doc["refYaw"]   = pf(req, "refYaw");
        doc["Ki1"]  = cp.Ki1;  doc["Ki2"]  = cp.Ki2;
        doc["Kx1"]  = cp.Kx1; doc["Kx2"]  = cp.Kx2;
        doc["Kx3"]  = cp.Kx3; doc["Kx4"]  = cp.Kx4;
        doc["Kx5"]  = cp.Kx5; doc["Kx6"]  = cp.Kx6;
        doc["Kx7"]  = cp.Kx7; doc["Kx8"]  = cp.Kx8;
        doc["openLoopPitch"] = cp.openLoopPitch;
        doc["openLoopYaw"]   = cp.openLoopYaw;

        File f = LittleFS.open("/parameters.json", "w");
        if (f) { serializeJson(doc, f); f.close(); }

        // Envia para taskControl (sem bloquear)
        if (ctrlParamsQueue) xQueueOverwrite(ctrlParamsQueue, &cp);

        req->send(200, "text/plain", "OK");
    });

    // ── Calibração ──────────────────────────────────────────────────────
    server.on("/api/calibration", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (LittleFS.exists("/calibration.json"))
            req->send(LittleFS, "/calibration.json", "application/json");
        else
            req->send(200, "application/json",
                      "{\"imuCalibrated\":false,\"motorCalibrated\":false}");
    });

    auto calibPost = [this](AsyncWebServerRequest* req, CalibCmd::Type t) {
        if (!calibCmdQueue) { req->send(503, "text/plain", "Queue not ready"); return; }
        CalibCmd cmd; cmd.type = t;
        xQueueOverwrite(calibCmdQueue, &cmd);
        req->send(200, "text/plain", "OK");
    };

    server.on("/api/calibrate/imu",      HTTP_POST, [=](AsyncWebServerRequest* r){ calibPost(r, CalibCmd::Type::IMU);      });
    server.on("/api/calibrate/motors",   HTTP_POST, [=](AsyncWebServerRequest* r){ calibPost(r, CalibCmd::Type::MOTORS);   });
    server.on("/api/calibrate/encoders", HTTP_POST, [=](AsyncWebServerRequest* r){ calibPost(r, CalibCmd::Type::ENCODERS); });
    server.on("/api/calibrate/reset",    HTTP_POST, [=](AsyncWebServerRequest* r){ calibPost(r, CalibCmd::Type::RESET);    });

    // ── Teste direto de motores ─────────────────────────────────────────
    server.on("/api/test/motors", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!motorCmdQueue) { req->send(503, "text/plain", "Queue not ready"); return; }
        int dp = constrain(pi(req, "dutyPitch"), -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
        int dy = constrain(pi(req, "dutyYaw"),   -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);
        MotorCmd cmd; cmd.mode = MotorCmd::Mode::TEST;
        cmd.dutyPitch = (float)dp; cmd.dutyYaw = (float)dy;
        xQueueOverwrite(motorCmdQueue, &cmd);
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/test/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (motorCmdQueue) {
            MotorCmd cmd; cmd.mode = MotorCmd::Mode::TEST;
            xQueueOverwrite(motorCmdQueue, &cmd);
        }
        req->send(200, "text/plain", "OK");
    });

    server.on("/api/test/control", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (motorCmdQueue) {
            MotorCmd cmd; cmd.mode = MotorCmd::Mode::CONTROL;
            xQueueOverwrite(motorCmdQueue, &cmd);
        }
        req->send(200, "text/plain", "OK");
    });

    // ── SSE ─────────────────────────────────────────────────────────────
    events.onConnect([](AsyncEventSourceClient* client) {
        client->send("{}", "telemetry", millis(), 1000);
    });
    server.addHandler(&events);

    // OTA via AP — acessível em 192.168.4.1/update
    ElegantOTA.begin(&server);
    ElegantOTA.onStart([this]() {
        if (motorCmdQueue) {
            MotorCmd stop;
            xQueueOverwrite(motorCmdQueue, &stop);
        }
        Serial.println("[OTA] Iniciando atualizacao — motores parados");
    });
    ElegantOTA.onEnd([](bool success) {
        Serial.printf("[OTA] %s — reiniciando\n", success ? "Sucesso" : "Falhou");
    });

    server.begin();
    Serial.println("Web server iniciado na porta 80.");
}
