#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <freertos/queue.h>
#include "env.h"
#include "MotorCmd.h"

class WebManager {
private:
    AsyncWebServer server;
    AsyncEventSource events;
    const char* ssid;
    const char* password;
    QueueHandle_t motorCmdQueue = nullptr;

public:
    WebManager(const char* ssid, const char* pass);
    void begin();
    void attachMotorQueue(QueueHandle_t queue);
    void sendTelemetry(const char* json);
};

#endif
