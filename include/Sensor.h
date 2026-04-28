#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

struct DadosSensores {
    float anguloPitch;
    float anguloYaw;
};

class Sensor {
  public:
    Sensor();
    void begin();
    DadosSensores lerDados();
};

#endif // SENSOR_H
