# Firmware de Controle — Drone de Bancada

Firmware de controle e telemetria desenvolvido pela equipe de Eletrônica e Sistemas Embarcados (ESE) para estabilização do drone de bancada em 1-DOF e 2-DOF, utilizando ESP32-S3.

---

## Hardware

| Componente | Modelo / Detalhe |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 (16 MB Flash, 8 MB PSRAM) |
| IMU | ICM42670P (I2C) |
| Encoders | Ópticos em quadratura — 600 PPR × 4 = 2400 pulsos/volta |
| Motores | DC com driver PWM bidirecional (RPWM/LPWM) |

---

## Arquitetura de Firmware

O sistema usa FreeRTOS em dual-core para isolar o controle crítico da telemetria:

![Arquitetura de Firmware](assets/fluxograma_pt.png)

| Task | Core | Frequência | Responsabilidade |
|---|---|---|---|
| `taskSensor` | 0 | 50 Hz | Lê IMU + encoders e distribui dados via filas |
| `taskTelemetry` | 0 | 50 Hz | Envia CSV via Serial e JSON via SSE para o browser |
| `taskControl` | 1 | 100 Hz | Lê dados da fila, calcula lei de controle e aciona motores |

A comunicação entre tasks é feita exclusivamente por **filas FreeRTOS** — sem variáveis globais compartilhadas.

```
IMU::read() ──► ctrlQueue  ──► taskControl   ──► Motor::setVelocidade()
           └──► telemQueue ──► taskTelemetry ──► Serial (CSV para MATLAB)
                                             └──► SSE /api/events (browser)
```

---

## Interface Web

O ESP32 sobe como **Access Point Wi-Fi**. Conecte-se à rede configurada e acesse `192.168.4.1` no browser.

| Página | Rota | Função |
|---|---|---|
| Config PID | `/` | Ajuste dos ganhos Kp, Ki, Kd de cada controlador |
| Teste de Motor | `/test` | Controle manual de duty cycle com telemetria em tempo real |

---

## Como começar

### Pré-requisitos

- [PlatformIO](https://platformio.org/) instalado (extensão VS Code ou CLI)
- ESP32-S3 conectado via USB

### 1. Clonar e configurar credenciais

```bash
git clone <url-do-repo>
cd integrador
cp include/env_example.h include/env.h
```

Edite `include/env.h` com o nome e senha da rede Wi-Fi que o ESP32 irá criar:

```cpp
#define AP_SSID     "NomeDaRede"
#define AP_PASSWORD "SenhaDaRede"
```

### 2. Compilar e gravar

```bash
pio run --target upload      # grava o firmware
pio run --target uploadfs    # grava as páginas web (rodar uma vez ou ao alterar data/)
pio device monitor           # abre o monitor serial (115200 baud)
```

---

## Integração — Equipe de Modelagem (MCM)

A lei de controle deve ser inserida em `src/main.cpp`, dentro de `taskControl`, nos trechos marcados com `TODO (MCM)`:

```cpp
float errPitch = 0.0f - last.pitch;
// TODO (MCM): substituir por PID
p->motorPitch->setVelocidade(errPitch);
```

Os dados disponíveis em `SensorData` são:

| Campo | Descrição |
|---|---|
| `pitch` | Ângulo pitch em graus (acelerômetro IMU) |
| `yaw` | Ângulo yaw em graus (integração giroscópio) |
| `encPitchDeg` | Ângulo do encoder de pitch em graus |
| `encYawDeg` | Ângulo do encoder de yaw em graus |

---

## Roadmap

- [x] Mapeamento de hardware e pinout
- [x] Ambiente FreeRTOS dual-core
- [x] Leitura de IMU (ICM42670P) e encoders em quadratura
- [x] Telemetria Serial (CSV) para MATLAB/Simulink
- [x] Interface web com configuração PID e teste de motor
- [ ] Integração da lei de controle (equipe MCM)
