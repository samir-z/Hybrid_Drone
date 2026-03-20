% =========================================================================
%                SIMULADOR: MASA - RESORTE - AMORTIGUADOR
% =========================================================================
clc; clear; close all;

% DEFINIMOS LA PLANTA FISICA
m = 1;  % Masa (Kg)
b = 10; % Amortiguamiento natural (Ns/m)
k = 20; % Resorte natural (N/m)
s = tf('s');
planta = 1/(m*s^2 + b*s +k);

% ESCENARIO P
Kp1 = 300;
C_P = pid(Kp1, 0, 0);
Sis_P = feedback(C_P * planta, 1);

% ESCENARIO PD
Kp2 = 300;
Kd2 = 1000;
C_PD = pid(Kp2, 0, Kd2);
Sis_PD = feedback(C_PD * planta, 1);

% ESCENARIO PID
Kp3 = 300;
Kd3 = 20;
Ki3 = 300;
C_PID = pid(Kp3, Ki3, Kd3);
Sis_PID = feedback(C_PID * planta, 1);

% GRAFICAMOS LA COMPETENCIA
figure('Name', 'Intuición PID', 'Position', [100,100,1000,600]);

% RESPUESTA DEL ESCALON (Queremos que la masa llegue a la posición 1)
t= 0:0.01:2; % 2 secounds simulation
[y_P, t] = step(Sis_P, t);
[y_PD, t] = step(Sis_PD, t);
[y_PID, t] = step(Sis_PID, t);

plot(t, y_P, 'r', 'LineWidth', 1.5); hold on;
plot(t, y_PD, 'b', 'LineWidth', 2);
plot(t, y_PID, 'g--', 'LineWidth', 2.5);
yline(1, 'k--', 'Setpoint', 'LineWidth', 1.5);

grid on;
title('Respuesta del Sistema: P vs PD vs PID');
xlabel('Tiempo (segundos)');
ylabel('Posición');
legend('Solo P (Rebota)', 'P + D (Estabilizado)', 'P + I + D (Perfecto)');