#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "Constants.h"
#include "Motor.h"
#include "Sensor.h"

struct TaskParams {
    QueueHandle_t queue;
    Sensor* sensor;
    Motor* motor;
};

void taskSensor(void* pvParams) {
    auto* p = (TaskParams*)pvParams;
    SensorData data;
    for (;;) {
        data = p->sensor->read();
        xQueueSend(p->queue, &data, portMAX_DELAY);
        Serial.printf("[Sensor] pitch=%.2f\n", data.pitch);
        vTaskDelay(pdMS_TO_TICKS(1000 / FREQ_SENSOR_HZ));
    }
}

void taskControl(void* pvParams) {
    auto* p = (TaskParams*)pvParams;
    TickType_t lastWake = xTaskGetTickCount();
    SensorData last = {};
    SensorData data;
    // first read: block until sensor delivers
    xQueueReceive(p->queue, &last, portMAX_DELAY);

    for (;;) {
        if (xQueueReceive(p->queue, &data, 0) == pdPASS)
            last = data;

        float error = 0.0f - last.pitch;
        float output = error * KP;
        p->motor->setVelocidade(output);

        Serial.printf("[Control] error=%.2f output=%.2f\n", error, output);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / FREQ_CONTROL_HZ));
    }
}

void setup() {
    Serial.begin(115200);

    static Motor motor(PIN_MOTOR_PITCH_PWM, PIN_MOTOR_PITCH_DIR);
    motor.begin();

    static Sensor sensor;
    sensor.begin();

    static QueueHandle_t queue = xQueueCreate(5, sizeof(SensorData));
    assert(queue != NULL);

    static TaskParams sensorParams = { queue, &sensor, nullptr };
    static TaskParams controlParams = { queue, nullptr, &motor };

    xTaskCreatePinnedToCore(taskSensor,  "sensor",  TASK_STACK_SIZE, &sensorParams,  1, NULL, 0);
    xTaskCreatePinnedToCore(taskControl, "control", TASK_STACK_SIZE, &controlParams, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }