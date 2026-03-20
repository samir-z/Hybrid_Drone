% =========================================================================
%               DISCRETIZACIÓN DEL CONTROLADOR (MÉTODO TUSTIN)
% =========================================================================
clc; clear; close all;

% PARAMETROS FISICOS ESTIMADOS
J = 0.001;
b = 0.01;
s = tf('s');
planta = 1 / (J*s^2 + b*s);

% PARAMETROS del PID Tuner
Kp = 0.25633;
Ki = 0.73634;
Kd = 0.018744;
Tf = 0.027145;

% ARRANCAMOS EL PID CONTINUO EN MATLAB
C_cont = pid(Kp, Ki, Kd, Tf);

% TUSTIN
Ts = 0.01; % El dron lee a 100Hz
C_disc = c2d(C_cont, Ts, 'tustin');

% VALIDACION
figure('Name', 'Validación: Continuo vs Discreto', 'Position', [100, 100, 800, 500]);
[y_cont, t_cont] = step(feedback(C_cont * planta, 1));      % El sistema continuo puro
planta_disc = c2d(planta, Ts, 'zoh');                       % Simulamos el PWM
[y_disc, t_disc] = step(feedback(C_disc * planta_disc, 1));

plot(t_cont, y_cont, 'b', 'LineWidth', 1.5);
hold on;
plot(t_disc, y_disc, 'r--', 'LineWidth', 2);
stairs(t_disc, y_disc, 'r', 'LineWidth', 1);                % Escalones rectangulares para ver exactamente lo que hace el STM32 

title('Validación de Discretización (Tustin + ZOH) a 100 Hz');
legend('PID Continuo (Física ideal)', 'PID Discreto (Nodos)', 'Salida ZOH STM32');
grid on;

% EXTREMOS LOS COEFICIENTES PARA EL CODIGO C
disp('====================================================');
disp('COEFICIENTES PARA FIRMWARE EN C (pid.c):');
disp('====================================================');
disp(['Kp = ', num2str(Kp)]);
disp(['Ki = ', num2str(Ki)]);
disp(['Kd = ', num2str(Kd)]);
disp(['N (1/Tf) = ', num2str(1/Tf)]);
disp(['Ts = ', num2str(Ts)]);