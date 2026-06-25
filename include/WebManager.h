#ifndef WEB_MANAGER_H
#define WEB_MANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <freertos/queue.h>
#include "env.h"
#include "MotorCmd.h"
#include "Calibration.h"

struct CalibCmd {
    enum class Type : uint8_t { NONE, IMU, MOTORS, ENCODERS, RESET } type = Type::NONE;
};

class WebManager {
private:
    AsyncWebServer server;
    AsyncEventSource events;
    const char* ssid;
    const char* password;
    QueueHandle_t motorCmdQueue   = nullptr;
    QueueHandle_t calibCmdQueue   = nullptr;
    QueueHandle_t ctrlParamsQueue = nullptr;   // depth 1, xQueueOverwrite

public:
    WebManager(const char* ssid, const char* pass);
    void begin();
    void attachMotorQueue(QueueHandle_t queue);
    void attachCalibQueue(QueueHandle_t queue);
    void attachCtrlParamsQueue(QueueHandle_t queue);
    void sendTelemetry(const char* json);
    void sendCalibStatus(const char* json);
};

#endif
