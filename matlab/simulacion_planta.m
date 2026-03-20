% =========================================================================
%           MODELADO FÍSICO Y SINTONIZACIÓN PID (DOMINIO CONTINUO)
% =========================================================================
clc; clear; close all;

% PARAMETROS FISICOS ESTIMADOS
J = 0.001;      % Momento de inercia (kg*m²) - Resistencia a girar
b = 0.01;       % Fricción aerodinamica del chasis

% DEFINIR LA FUNCION DE TRANSFERENCIA
s = tf('s');
planta = 1 / (J*s^2 + b*s);

disp('Modelo de la Planta P(s):');
planta

pidTuner(planta, 'PID')