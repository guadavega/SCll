clc; clear all; close all;

% Caso de estudio 2. Sistema lineal de cuatro variables de estado.

% Parámetros del sistema
a = 0.07;
b = 5;
c = 150;
w = 9;

% Matrices de estado continuas
A = [-a a 0 0;   
     0 0 1 0;
     w^2 -w^2 0 0;
     c 0 0 0];
B = [0; 0; b*w^2; 0];
C = [0 0 0 1; 0 1 0 0];

% Variables de estado
alpha = 0;      % condiciones iniciales
phi = 0;
phi_p = 0;
high = 500;     % altura de inicio
u = 0;          % acción de control
uu = 0;
x = [alpha; phi; phi_p; high];

% Polos deseados para el controlador
p1 = -15 + 15i;
p2 = -15 - 15i;
p3 = -0.5 + 0.5i;
p4 = -0.5 - 0.5i;

% Controlabilidad
Mc = [B A*B A^2*B A^3*B]; 
controlabilidad = rank(Mc); 

numVE = size(x);

if(numVE(1) == controlabilidad)
    fprintf('\nSistema controlable')
else
    fprintf('\nSistema NO controlable')
end

% Controlador continuo
K = acker(A, B, [p1 p2 p3 p4]);

% Ganancia de prealimentación
G = -inv(C(1,:) * inv(A - B * K) * B);

% Controlador LQR 
Q = diag([1 1000000 1 1]);
R = 1000000;
Klqr = lqr(A, B, Q, R);
G_lqr = -inv(C(1,:)*inv(A-B*Klqr)*B);

eig(A - B*Klqr)

% Observador
Ao = A';
Bo = C';
Co = B';

% Observador LQR

Qo=100*diag([100 100 10 100]);    
Ro=1;
Ko = lqr(Ao, Bo, Qo, Ro)';

T = 70;         % Tiempo de simulación

% Se toma que la frecuencia de muestreo sea 20 veces más grande, entonces
% Ts=1/(9x20)=0.0055 segundos
Ts = 1e-4;      % Tiempo de muestreo
t = 0:Ts:(T-Ts);  % Vector tiempo
pasos = T/Ts;

ref = 100;      % Referencia de 100 mtrs de altura


xobs = [0; 0; 0; high];

alpha_hist = zeros(size(t));     % Variables para guardar el estado del sistema 
phi_hist = zeros(size(t));       % Se actualizan al finalizar el bucle
phi_p_hist = zeros(size(t));
high_hist = zeros(size(t));
u_hist = zeros(size(t));
alpha_obs_hist = zeros(size(t));
phi_obs_hist = zeros(size(t));
phi_p_obs_hist = zeros(size(t));
high_obs_hist = zeros(size(t));

for i = 1:pasos
    % Controlador
    %u = -Klqr * xobs + G_lqr * ref;   
     u = -K * xobs + G * ref;
     
    % Zona muerta
    if abs(u) < 0.1
        uu = 0;
    else
        uu = sign(u) * (abs(u) - 0.1);
    end
    
    % Variables del sistema lineal
    alpha_hist(i) = x(1);
    phi_hist(i) = x(2);
    phi_p_hist(i) = x(3);
    high_hist(i) = x(4);
    
    % Sistema lineal
    xp = A * x + B * uu;
    x = x + Ts * xp;
    
    % Observador
    y = C * x;          % salida real
    y_obs = C * xobs;   % salida estimada por el observador
    e = y - y_obs;      % error de estimación
    x_obs_p = A * xobs + B * uu + Ko * e; % dinámica del observador
    xobs = xobs + Ts * x_obs_p;
    
    % Se guardan las estimaciones del observador
    alpha_obs_hist(i) = xobs(1);
    phi_obs_hist(i) = xobs(2);
    phi_p_obs_hist(i) = xobs(3);
    high_obs_hist(i) = xobs(4);
    
    u_hist(i) = uu;
end


figure;
subplot(3, 2, 1);
hold on
plot(t, alpha_hist, 'b');
plot(t, alpha_obs_hist, 'r--');
hold off
title('Angulo con la horizontal \alpha');
legend({'Real', 'Observado'});
xlabel('Tiempo (seg.)');
ylabel('rad');
grid on;

subplot(3, 2, 2);
hold on
plot(t, phi_hist, 'b');
plot(t, phi_obs_hist, 'r--');
hold off
title('Angulo de cabeceo \phi');
legend({'Real', 'Observado'});
xlabel('Tiempo (seg.)');
ylabel('Velocidad (m/s)');
grid on;

subplot(3, 2, 3);
hold on
plot(t, phi_p_hist, 'b');
plot(t, phi_p_obs_hist, 'r--');
hold off
title('Velocidad de angulo de cabeceo \phi_p');
legend({'Real', 'Observado'});
xlabel('Tiempo (seg.)');
ylabel('Posicion angular (Rad/s)');
grid on;

subplot(3, 2, 4);
hold on
plot(t, high_hist, 'b');
plot(t, high_obs_hist, 'r--');
hold off
title('Altura h');
legend({'Real', 'Observado'});
xlabel('Tiempo (seg.)');
ylabel('metros');
grid on;

subplot(3, 1, 3);
hold on
plot(t, u_hist, 'm');
hold off
title('Accion de control u_t');
xlabel('Tiempo (seg.)');
ylabel('V');
grid on;
