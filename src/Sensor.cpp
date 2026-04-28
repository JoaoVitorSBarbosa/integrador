#include "Sensor.h"
#include "Constants.h"

Sensor::Sensor() {
    // Construtor caso precise inicializar algo
}

void Sensor::begin() {
    // Inicialização de hardware do sensor
    // Ex: Wire.begin(); para I2C
}

DadosSensores Sensor::lerDados() {
    DadosSensores dados;
    // Lógica de leitura real do sensor vai aqui
    // exemplo fake:
    dados.anguloPitch = 15.5; 
    dados.anguloYaw = 0.0;
    return dados;
}
