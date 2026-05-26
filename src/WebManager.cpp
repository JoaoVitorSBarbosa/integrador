#include "WebManager.h"

WebManager::WebManager(const char* apSsid, const char* apPassword)
    : server(80), events("/api/events") {
    this->ssid     = apSsid;
    this->password = apPassword;
}

void WebManager::attachMotors(Motor* pitch, Motor* yaw) {
    motorPitch = pitch;
    motorYaw   = yaw;
}

void WebManager::sendTelemetry(const char* json) {
    events.send(json, "telemetry", millis());
}

void WebManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("Erro ao montar o LittleFS!");
        return;
    }
    Serial.println("LittleFS montado com sucesso.");
    Serial.println("Iniciando modo Access Point (AP)...");

    WiFi.softAP(ssid, password);
    IPAddress apIp = WiFi.softAPIP();
    Serial.print("Endereço IP do AP: ");
    Serial.println(apIp);

    // --- Páginas ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/test", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/test.html", "text/html");
    });

    // --- PID params ---
    server.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/parameters.json", "application/json");
    });

    server.on("/api/params", HTTP_POST, [](AsyncWebServerRequest* request) {
        auto get = [&](const char* key) -> String {
            return request->hasParam(key, true)
                ? request->getParam(key, true)->value()
                : "0";
        };

        String json =
            "[\n"
            "    {\n"
            "        \"Kp1\": \"" + get("Kp1") + "\",\n"
            "        \"Kd1\": \"" + get("Kd1") + "\",\n"
            "        \"Ki1\": \"" + get("Ki1") + "\"\n"
            "    },\n"
            "    {\n"
            "        \"Kp2\": \"" + get("Kp2") + "\",\n"
            "        \"Kd2\": \"" + get("Kd2") + "\",\n"
            "        \"Ki2\": \"" + get("Ki2") + "\"\n"
            "    }\n"
            "]";
        Serial.println(json);

        File f = LittleFS.open("/parameters.json", "w");
        if (!f) {
            request->send(500, "text/plain", "Erro ao abrir arquivo");
            return;
        }
        f.print(json);
        f.close();
        request->send(200, "text/plain", "OK");
    });

    // --- Teste de motor ---
    server.on("/api/test/motor", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (!motorPitch || !motorYaw) {
            request->send(503, "text/plain", "Motores nao inicializados");
            return;
        }

        int duty  = request->hasParam("duty",  true) ? request->getParam("duty",  true)->value().toInt() : 0;
        int motor = request->hasParam("motor", true) ? request->getParam("motor", true)->value().toInt() : 0;

        duty = constrain(duty, -MOTOR_PWM_MAX_DUTY, MOTOR_PWM_MAX_DUTY);

        if (motor == 0)
            motorPitch->setVelocidade(duty);
        else
            motorYaw->setVelocidade(duty);

        request->send(200, "text/plain", "OK");
    });

    server.on("/api/test/stop", HTTP_POST, [this](AsyncWebServerRequest* request) {
        if (motorPitch) motorPitch->parar();
        if (motorYaw)   motorYaw->parar();
        request->send(200, "text/plain", "OK");
    });

    // --- SSE telemetria ---
    events.onConnect([](AsyncEventSourceClient* client) {
        client->send("{}", "telemetry", millis(), 1000);
    });
    server.addHandler(&events);

    server.begin();
    Serial.println("Servidor Web iniciado na porta 80.");
}
