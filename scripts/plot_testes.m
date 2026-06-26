% plot_testes.m
% Analisa o CSV gerado pela rotina de testes do firmware (open-loop / closed-loop).
%
% Colunas do CSV (formato atual do firmware):
%   t_ms, dutyPitch[PWM], dutyYaw[PWM],
%   ax[m/s2], ay[m/s2], az[m/s2],
%   velPitch[rad/s], velYaw[rad/s],
%   pitch[rad], yaw[rad], encPitch[rad], encYaw[rad]

clear; clc; close all;

R2D = 180 / pi;   % fator rad → graus

%% --- Carrega arquivo ---
[file, path] = uigetfile('*.csv', 'Selecione o arquivo testes.csv');
if isequal(file, 0)
    disp('Nenhum arquivo selecionado. Encerrando.');
    return;
end

opts = detectImportOptions(fullfile(path, file));
opts.VariableNamingRule = 'preserve';
raw = readtable(fullfile(path, file), opts);

% Valida número de colunas e renomeia para nomes simples
EXPECTED_COLS = 12;
if width(raw) ~= EXPECTED_COLS
    error('CSV tem %d colunas; esperadas %d.\nColunas encontradas: %s', ...
          width(raw), EXPECTED_COLS, strjoin(raw.Properties.VariableNames, ', '));
end
raw.Properties.VariableNames = { ...
    't_ms','dutyPitch','dutyYaw', ...
    'ax','ay','az', ...
    'velPitch','velYaw', ...
    'pitch','yaw','encPitch','encYaw'};

% Extrai variáveis
t         = (raw.t_ms - raw.t_ms(1)) / 1000;   % [s]

pitch_deg = raw.pitch    * R2D;
yaw_deg   = raw.yaw      * R2D;
encPitch  = raw.encPitch * R2D;
encYaw    = raw.encYaw   * R2D;

velPitch  = raw.velPitch;   % [rad/s]
velYaw    = raw.velYaw;

ax        = raw.ax;
ay        = raw.ay;
az        = raw.az;

dutyPitch = raw.dutyPitch;
dutyYaw   = raw.dutyYaw;

%% --- Figura 1: Ângulos (IMU + encoder) ---
figure('Name', 'Ângulos', 'NumberTitle', 'off', 'Position', [100 100 900 600]);

subplot(2, 1, 1);
plot(t, pitch_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encPitch,  'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Pitch');
legend('IMU (acelerômetro)', 'Encoder', 'Location', 'best');
grid on;

subplot(2, 1, 2);
plot(t, yaw_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encYaw,  'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Yaw');
legend('IMU (giroscópio)', 'Encoder', 'Location', 'best');
grid on;

sgtitle('Ângulos ao longo do tempo');

%% --- Figura 2: Velocidades angulares ---
figure('Name', 'Velocidades', 'NumberTitle', 'off', 'Position', [100 100 900 600]);

subplot(2, 1, 1);
plot(t, velPitch * R2D, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade angular (°/s)');
title('Pitch — \omega');
grid on;

subplot(2, 1, 2);
plot(t, velYaw * R2D, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade angular (°/s)');
title('Yaw — \omega');
grid on;

sgtitle('Velocidades angulares ao longo do tempo');

%% --- Figura 3: Acelerômetro ---
figure('Name', 'Acelerômetro', 'NumberTitle', 'off', 'Position', [100 100 900 500]);

plot(t, ax, 'r-', 'LineWidth', 1.2); hold on;
plot(t, ay, 'g-', 'LineWidth', 1.2);
plot(t, az, 'b-', 'LineWidth', 1.2);
xlabel('Tempo (s)'); ylabel('Aceleração (m/s²)');
title('Leituras do Acelerômetro');
legend('ax', 'ay', 'az', 'Location', 'best');
grid on;

%% --- Figura 4: Sinal de controle (duty cycle) ---
figure('Name', 'Sinal de Controle', 'NumberTitle', 'off', 'Position', [100 100 900 500]);

subplot(2, 1, 1);
stairs(t, dutyPitch, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Duty PWM');
title('Sinal de Controle — Pitch');
ylim([-260 260]); grid on;

subplot(2, 1, 2);
stairs(t, dutyYaw, 'r-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Duty PWM');
title('Sinal de Controle — Yaw');
ylim([-260 260]); grid on;

sgtitle('Duty Cycle dos Motores');

%% --- Figura 5: Painel completo ---
figure('Name', 'Painel Completo', 'NumberTitle', 'off', 'Position', [80 80 1200 750]);

subplot(2, 3, 1);
plot(t, pitch_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encPitch,  'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Ângulo de Pitch');
legend('IMU', 'Encoder', 'Location', 'best'); grid on;

subplot(2, 3, 2);
plot(t, yaw_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encYaw,  'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Ângulo de Yaw');
legend('IMU', 'Encoder', 'Location', 'best'); grid on;

subplot(2, 3, 3);
plot(t, ax, 'r-', t, ay, 'g-', t, az, 'b-', 'LineWidth', 1.2);
xlabel('Tempo (s)'); ylabel('m/s²');
title('Acelerômetro');
legend('ax','ay','az','Location','best'); grid on;

subplot(2, 3, 4);
plot(t, velPitch * R2D, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('°/s');
title('Velocidade Angular — Pitch'); grid on;

subplot(2, 3, 5);
plot(t, velYaw * R2D, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('°/s');
title('Velocidade Angular — Yaw'); grid on;

subplot(2, 3, 6);
stairs(t, dutyPitch, 'b-', 'LineWidth', 1.5); hold on;
stairs(t, dutyYaw,   'r-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Duty PWM');
title('Sinal de Controle');
legend('Pitch','Yaw','Location','best');
ylim([-260 260]); grid on;

sgtitle('Análise de Teste — Drone de Bancada');
