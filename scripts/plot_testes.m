% plot_testes.m
% Analisa o CSV gerado pela rotina de testes do firmware (open-loop / closed-loop).
%
% Colunas do CSV:
%   t_ms, mode, pitch_deg, yaw_deg, pitchDot, yawDot,
%   encPitch_deg, encYaw_deg, uPitch, uYaw

clear; clc; close all;

%% --- Carrega arquivo ---
[file, path] = uigetfile('*.csv', 'Selecione o arquivo testes.csv');
if isequal(file, 0)
    disp('Nenhum arquivo selecionado. Encerrando.');
    return;
end

raw = readtable(fullfile(path, file));

% Tempo em segundos a partir do zero
t = (raw.t_ms - raw.t_ms(1)) / 1000;   % [s]

pitch_deg    = raw.pitch_deg;
yaw_deg      = raw.yaw_deg;
pitchDot     = raw.pitchDot;
yawDot       = raw.yawDot;
encPitch_deg = raw.encPitch_deg;
encYaw_deg   = raw.encYaw_deg;

%% --- Figura 1: Ângulos (IMU + encoder) ---
figure('Name', 'Ângulos', 'NumberTitle', 'off', 'Position', [100 100 900 600]);

subplot(2, 1, 1);
plot(t, pitch_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encPitch_deg, 'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
title('Pitch');
legend('IMU (acelerômetro)', 'Encoder', 'Location', 'best');
grid on;

subplot(2, 1, 2);
plot(t, yaw_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encYaw_deg, 'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Ângulo (°)');
title('Yaw');
legend('IMU (giroscópio)', 'Encoder', 'Location', 'best');
grid on;

sgtitle('Ângulos ao longo do tempo');

%% --- Figura 2: Velocidades angulares ---
figure('Name', 'Velocidades', 'NumberTitle', 'off', 'Position', [100 100 900 600]);

subplot(2, 1, 1);
plot(t, pitchDot, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Velocidade angular (°/s)');
title('Pitch — \omega');
grid on;

subplot(2, 1, 2);
plot(t, yawDot, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Velocidade angular (°/s)');
title('Yaw — \omega');
grid on;

sgtitle('Velocidades angulares ao longo do tempo');

%% --- Figura 3: Painel completo (4 subplots) ---
figure('Name', 'Painel Completo', 'NumberTitle', 'off', 'Position', [100 100 1100 700]);

subplot(2, 2, 1);
plot(t, pitch_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encPitch_deg, 'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Ângulo de Pitch');
legend('IMU', 'Encoder', 'Location', 'best');
grid on;

subplot(2, 2, 2);
plot(t, yaw_deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, encYaw_deg, 'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Ângulo (°)');
title('Ângulo de Yaw');
legend('IMU', 'Encoder', 'Location', 'best');
grid on;

subplot(2, 2, 3);
plot(t, pitchDot, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade angular (°/s)');
title('Velocidade de Pitch (\omega)');
grid on;

subplot(2, 2, 4);
plot(t, yawDot, 'b-', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Velocidade angular (°/s)');
title('Velocidade de Yaw (\omega)');
grid on;

sgtitle('Análise de teste — Drone de bancada');
