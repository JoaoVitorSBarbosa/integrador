#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <LittleFS.h>
#include "env.h"
#include "Motor.h"

class WebManager {
private:
    AsyncWebServer server;
    AsyncEventSource events;
    const char* ssid;
    const char* password;
    Motor* motorPitch = nullptr;
    Motor* motorYaw   = nullptr;

public:
    WebManager(const char* ssid, const char* pass);
    void begin();
    void attachMotors(Motor* pitch, Motor* yaw);
    void sendTelemetry(const char* json);
};

#endif
