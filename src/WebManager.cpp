#include "WebManager.h"

// Inicializa o servidor na porta 80 e define as credenciais

WebManager::WebManager(const char* apSsid, const char* apPassword)
    : server(80) {
  this->ssid = apSsid;
  this->password = apPassword;
}

void WebManager::begin() {
  if (!LittleFS.begin(true)) {  // O 'true' tenta formatar se houver erro grave

    Serial.println("Erro ao montar o LittleFS!");
    return;
  }

  Serial.println("LittleFS montado com sucesso.");
  Serial.println("Iniciando modo Access Point (AP)...");

  // Configura o ESP32 como Access Point

  WiFi.softAP(ssid, password);
  IPAddress apIp = WiFi.softAPIP();
  Serial.print("Endereço IP do AP: ");
  Serial.println(apIp);

  // Rota padrao, cai direto na pagina de login
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

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

  server.begin();

  Serial.println("Servidor Web iniciado na porta 80.");
}