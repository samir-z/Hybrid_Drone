% =========================================================================
%                     Script de Calibración MPU6050
% =========================================================================
clc; clear; close all;

% LEER LOS DATOS (Ignorando las etiquetas de texto)
% El formato es: T:123 AX:1.23 AY:2.34 ...
disp('Leyendo reposo.csv...');
fid = fopen('reposo.csv', 'r');
datos = textscan(fid, 'T:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f', 'MultipleDelimsAsOne', true);
fclose(fid);

% EXTRAER VECTORES
T   = datos{1};
ax  = datos{2}; ay = datos{3}; az = datos{4};
gx  = datos{5}; gy = datos{6}; gz = datos{7};

% CALCULO DE ERRORES MEDIOS
% El acelerómetro Z deberia medir 9.80665 m/s² (gravedad)
mean_ax = mean(ax);
mean_ay = mean(ay);
mean_az = mean(az) - 9.80665;

mean_gx = mean(gx);
mean_gy = mean(gy);
mean_gz = mean(gz);

% CONVERSION A LSB (DATOS CRUDOS DEL MPU6050)
% Según la libreria mpu6050.h:
% Accel Sensibilidad: 8192 LSB/g (y 1g = 9.80665 m/s²) -> Factor = 8192 / 9.80665
% Gyro Sensibilidad: 65.5 LSB/(°/s)

accel_factor = 8192.0 / 9.80665;
gyro_factor  = 65.5;

offset_ax_lsb = round(mean_ax * accel_factor);
offset_ay_lsb = round(mean_ay * accel_factor);
offset_az_lsb = round(mean_az * accel_factor);

offset_gx_lsb = round(mean_gx * gyro_factor);
offset_gy_lsb = round(mean_gy * gyro_factor);
offset_gz_lsb = round(mean_gz * gyro_factor);

% IMPRIMIR CÓDIGO PARA PEGAR EN EL FIRMWARE
fprintf('\n=== COPIAR Y PEGAR ESTO EN mpu6050.h ===\n');
fprintf('#define MPU6050_OFFSET_AX       (%d)\n', offset_ax_lsb);
fprintf('#define MPU6050_OFFSET_AY       (%d)\n', offset_ay_lsb);
fprintf('#define MPU6050_OFFSET_AZ       (%d)\n', offset_az_lsb);
fprintf('#define MPU6050_OFFSET_GX       (%d)\n', offset_gx_lsb);
fprintf('#define MPU6050_OFFSET_GY       (%d)\n', offset_gy_lsb);
fprintf('#define MPU6050_OFFSET_GZ       (%d)\n', offset_gz_lsb);
fprintf('===========================================\n');

% =========================================================================
%           VISUALIZACIÓN DE LA CALIBRACIÓN (Crudo vs Corregido)
% =========================================================================

% Calcular los vectores corregidos simulando lo que hará el STM32
ax_corr = ax - mean_ax;
ay_corr = ay - mean_ay;
az_corr = az - (mean(az) - 9.80665); % Al Z le restamos el error, pero conservamos la gravedad

gx_corr = gx - mean_gx;
gy_corr = gy - mean_gy;
gz_corr = gz - mean_gz;

% Crear la ventana de la figura (grande para ver detalles)
figure('Name', 'Calibración MPU6050', 'Position', [100, 100, 1200, 800]);

% ===== COLUMNA IZQUIERDA: ACELERÓMETROS =====
subplot(3, 2, 1);
plot(T, ax, 'r', 'LineWidth', 1); hold on;
plot(T, ax_corr, 'b', 'LineWidth', 1.5);
grid on; title('Acelerómetro X'); ylabel('m/s^2'); legend('Crudo', 'Corregido');

subplot(3, 2, 3);
plot(T, ay, 'r', 'LineWidth', 1); hold on;
plot(T, ay_corr, 'b', 'LineWidth', 1.5);
grid on; title('Acelerómetro Y'); ylabel('m/s^2');

subplot(3, 2, 5);
plot(T, az, 'r', 'LineWidth', 1); hold on;
plot(T, az_corr, 'b', 'LineWidth', 1.5);
grid on; title('Acelerómetro Z'); ylabel('m/s^2'); xlabel('Tiempo (ms)');

% ===== COLUMNA DERECHA: GIROSCOPIOS =====
subplot(3, 2, 2);
plot(T, gx, 'r', 'LineWidth', 1); hold on;
plot(T, gx_corr, 'b', 'LineWidth', 1.5);
grid on; title('Giroscopio X'); ylabel('º/s'); legend('Crudo', 'Corregido');

subplot(3, 2, 4);
plot(T, gy, 'r', 'LineWidth', 1); hold on;
plot(T, gy_corr, 'b', 'LineWidth', 1.5);
grid on; title('Giroscopio Y'); ylabel('º/s');

subplot(3, 2, 6);
plot(T, gz, 'r', 'LineWidth', 1); hold on;
plot(T, gz_corr, 'b', 'LineWidth', 1.5);
grid on; title('Giroscopio Z'); ylabel('º/s'); xlabel('Tiempo (ms)');