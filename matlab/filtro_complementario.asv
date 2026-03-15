% =========================================================================
%                     Script de Filtro Complementario
% =========================================================================
clc; clear; close all;

% LEER LOS DATOS DE MOVIMIENTO
disp('Leyendo movimiento.csv...');
fid = fopen('movimiento.csv', 'r');
datos = textscan(fid, 'T:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f', 'MultipleDelimsAsOne', true);
fclose(fid);

T = datos{1};
ax = datos{2}; ay = datos{3}; az = datos{4};
gx = datos{5}; gy = datos{6}; gz = datos{7};
N = length(T);

% INICIALIZAR VECTORES DE MEMORIA
pitch_accel = zeros(N, 1);
pitch_gyro  = zeros(N, 1);
pitch_cf    = zeros(N, 1);

% CALCULO DEL ACELEROMETRO
% Usamos atan2d que ya devuelve el resultado en grados.
pitch_accel = atan2d(ay, az);

% CONDICIONES INICIALES
% El giroscopio no sabe dónde empezó, así que le creemos al acelerómetro en el ms 0.
pitch_gyro(1) = pitch_accel(1);
pitch_cf(1)   = pitch_accel(1);

alpha = 0.98; % Coeficiente de fusión

% EL BUCLE DE TIEMPO REAL (Simulando el STM32)
for i = 2:N
    % Calcular dt dinámico en segundos (debería ser ~0.01s)
    dt = (T(i) - T(i-1)) / 1000.0; 
    
    % Protección contra errores de lectura (si dt es 0 o irreal)
    if dt <= 0 || dt > 0.1
        dt = 0.01; 
    end
    
    % Integración pura del giroscopio (Acumula el error / Drift)
    pitch_gyro(i) = pitch_gyro(i-1) + (gx(i) * dt);
    
    % El Filtro Complementario
    pitch_cf(i) = alpha * (pitch_cf(i-1) + (gx(i) * dt)) + (1 - alpha) * pitch_accel(i);
end

% VISUALIZACIÓN DEL EXPERIMENTO
figure('Name', 'Filtro Complementario (Pitch)', 'Position', [100, 100, 1000, 600]);

% Graficamos las tres señales superpuestas
plot(T, pitch_accel, 'r', 'LineWidth', 1); hold on;
plot(T, pitch_gyro, 'b', 'LineWidth', 1.5);
plot(T, pitch_cf, 'g', 'LineWidth', 2);

grid on;
title('Fusión de Sensores: Eje Pitch');
xlabel('Tiempo (ms)');
ylabel('Ángulo (Grados)');
legend('Acelerómetro (Ruidoso)', 'Giroscopio (Drift)', 'Filtro Complementario (Perfecto)');