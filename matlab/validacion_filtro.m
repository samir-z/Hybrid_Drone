% =========================================================================
%                     Script de Validación del filtro C
% =========================================================================
clc; clear; close all;

% LEER LOS DATOS CRUDOS Y FILTRADOS
disp('Leyendo validacion.csv...');
fid = fopen('validacion_0,98.csv', 'r');
datos = textscan(fid, 'T:%f,Ax:%f,Ay:%f,Az:%f,Gx:%f,Gy:%f,Gz:%f,P:%f,R:%f', 'MultipleDelimsAsOne', true);
fclose(fid);

% Extraer las columnas de datos en variables separadas
T  = datos{1};
ax = datos{2}; ay = datos{3}; az = datos{4};
gx = datos{5}; gy = datos{6}; gz = datos{7};
P  = datos{8}; R  = datos{9};
N  = length(T);

% INICIALIZAR VECTORES DE MEMORIA
pitch_accel = zeros(N, 1);
pitch_gyro  = zeros(N, 1);
pitch_cf    = zeros(N, 1);
roll_accel  = zeros(N, 1);
roll_gyro   = zeros(N, 1);
roll_cf     = zeros(N, 1);

% CALCULO DEL ACELEROMETRO
% Usamos atan2d que ya devuelve el resultado en grados.
pitch_accel = atan2d(ay, az);
roll_accel = atan2d(ax, az) * -1;

% CONDICIONES INICIALES
% El giroscopio no sabe dónde empezó, así que le creemos al acelerómetro en el ms 0.
pitch_gyro(1) = pitch_accel(1);
pitch_cf(1)   = pitch_accel(1);
roll_gyro(1) = roll_accel(1);
roll_cf(1)   = roll_accel(1);

alpha = 0.98; % Coeficiente de fusión

% EL BUCLE DE TIEMPO REAL
for i = 2:N
    % Calcular dt dinámico en segundos (debería ser ~0.01s)
    dt = (T(i) - T(i-1)) / 1000.0; 
    
    % Protección contra errores de lectura (si dt es 0 o irreal)
    if dt <= 0 || dt > 0.1
        dt = 0.01; 
    end
    
    % Integración pura del giroscopio (Acumula el error / Drift)
    pitch_gyro(i) = pitch_gyro(i-1) + (gx(i) * dt);
    roll_gyro(i)  = roll_gyro(i-1) + (gy(i) * dt);
    
    % El Filtro Complementario
    pitch_cf(i) = alpha * (pitch_cf(i-1) + (gx(i) * dt)) + (1 - alpha) * pitch_accel(i);
    roll_cf(i) = alpha * (roll_cf(i-1) + (gy(i) * dt)) + (1 - alpha) * roll_accel(i);
end

% CALCULAMOS EL ERROR
delta_error_pitch = abs(pitch_cf - P);
delta_error_roll  = abs(roll_cf - R);

% COMPARATIVA DE SOFTWARE VS HARDWARE
figure('Name', 'STM32 VS MATLAB', 'Position', [100, 100, 1000, 600]);

% Subplot 1: Pitch
subplot(2,2,1);
plot(T, pitch_cf, 'r', 'LineWidth', 2); hold on;
plot(T, P, 'b', 'LineWidth', 1);
title('Pitch: Filtro Complementario vs Hardware');
xlabel('Tiempo (ms)');
ylabel('Grados');
legend('Pitch CF', 'Pitch Hardware (P)');
grid on;
hold off;

% Subplot 2: Roll
subplot(2,2,2);
plot(T, roll_cf, 'r', 'LineWidth', 2); hold on;
plot(T, R, 'b', 'LineWidth', 1);
title('Roll: Filtro Complementario vs Hardware');
xlabel('Tiempo (ms)');
ylabel('Grados');
legend('Roll CF', 'Roll Hardware (R)');
grid on;
hold off;

% Subplot 3: Error Pitch
subplot(2,2,3);
plot(T, delta_error_pitch, 'g', 'LineWidth', 2); hold on;
title('Pitch: Comparativa del error entre MATLAB y STM32');
xlabel('Tiempo (ms)');
ylabel('Error °');

% Subplot 4: Error Roll
subplot(2,2,4);
plot(T, delta_error_roll, 'g', 'LineWidth', 2); hold on;
title('Roll: Comparativa del error entre MATLAB y STM32');
xlabel('Tiempo (ms)');
ylabel('Error °');