#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <LittleFS.h>
#include "env.h"

class WebManager {
 private:
  AsyncWebServer server;
  const char* ssid;
  const char* password;

 public:

  WebManager(const char* ssid, const char* pass);

  // Método de inicialização
  void begin();
};

#endif