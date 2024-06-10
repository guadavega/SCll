clc; clear all; close all;

% Parámetros del sistema
m = 0.1;
m2 = 10 * m;
M = 1.5;   % masa
l = 1.6;   % longitud
F = 0.1;   % fuerza
g = 9.8;   % gravedad

% Matrices del sistema linealizado
A = [0 1 0 0;
     0 -F/M -(m*g)/M 0;
     0 0 0 1;
     0 F/(M*l) (M+m)*g/(M*l) 0];
 
Am2 = [0 1 0 0;
       0 -F/M -(m2*g)/M 0;
       0 0 0 1;
       0 F/(M*l) (M+m2)*g/(M*l) 0];
 
B = [0; 1/M; 0; -1/(M*l)];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% Controlador LQR
Q = diag([100 1 10 1]);  
R = 0.01;                
K = lqr(A, B, Q, R);
K2 = lqr(Am2, B, Q, R);        

% Ganancia de prealimentación
G = -inv(C(1,:) * inv(A - B * K) * B);
G2 = -inv(C(1,:) * inv(Am2 - B * K2) * B);

% Simulación
T = 15;          % tiempo de simulación
delta_T = 1e-4;  
t = 0:delta_T:T;      % vector de tiempo

% Condiciones iniciales
x = [0; 0; pi; 0];
ref = 10;  % Referencia de desplazamiento

% Variables para guardar la historia de estados y entradas
x_hist = zeros(length(t), 4);
u_hist = zeros(length(t), 1);

% Banderas para controlar el cambio de masa y referencia
m2_changed = false;
reference_changed = false;

for i = 1:length(t)
    % Controlador LQR
    if m2_changed
        u = -K2 * x + G2 * ref;
    else
        u = -K * x + G * ref;
    end
    
    % Dinámica del sistema
    if m2_changed
        dx = Am2 * x + B * u;
    else
        dx = A * x + B * u;
    end
    x = x + dx * delta_T;
    
    % Guardar variables
    x_hist(i, :) = x';
    u_hist(i) = u;
    
    % Cambiar masa y referencia después de alcanzar el desplazamiento de 10
    if x(1) >= 10 && ~m2_changed
        m2_changed = true;
        ref = 0;  % Cambiar referencia para regresar al origen
    end
end

% Graficar resultados
figure;
subplot(3, 1, 1);
plot(t, x_hist(:, 1));
title('Desplazamiento \delta');
xlabel('Tiempo (s)');
ylabel('Desplazamiento (m)');
grid on;

subplot(3, 1, 2);
plot(t, x_hist(:, 3));
title('Ángulo \phi');
xlabel('Tiempo (s)');
ylabel('Ángulo (rad)');
grid on;

subplot(3, 1, 3);
plot(t, u_hist);
title('Acción de control u');
xlabel('Tiempo (seg)');
ylabel('Fuerza (f)');
grid on;
