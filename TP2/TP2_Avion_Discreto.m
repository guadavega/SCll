clc; clear all; close all;

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
D = 0;

% Polos deseados para el controlador continuo
p1 = -15 + 15i;
p2 = -15 - 15i;
p3 = -0.5 + 0.5i;
p4 = -0.5 - 0.5i;

% Tiempo de muestreo
Ts = 0.05;      % Tiempo de muestreo

% Conversión de las matrices del sistema al dominio discreto
sys_d = c2d(ss(A, B, C, D), Ts);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

% Convertir los polos continuos a discretos
p_discrete = exp([p1 p2 p3 p4] * Ts);

% Diseño del controlador discreto usando los polos discretos
K_disc = place(Ad, Bd, p_discrete);

% Ganancia de prealimentación en tiempo discreto
G_disc = -inv(Cd(1,:) * inv(Ad - Bd * K_disc) * Bd);

% Diseño del observador LQR discreto
Qo = diag([10 10 1 10]);     
Ro = 1;
Ko_disc = dlqr(Ad', Cd', Qo, Ro);
Ko_disc = Ko_disc';      % Ajuste de dimensiones

% Variables de simulación
T = 70;         % Tiempo de simulación
t = 0:Ts:(T-Ts);  % Vector tiempo
pasos = T/Ts;

ref = 100;      % Referencia de 100 mtrs de altura

alpha = 0;      % Condiciones iniciales
phi = 0;
phi_p = 0;
high = 500;     % Altura de inicio
u = 0;          % Acción de control
uu = 0;
x = [alpha; phi; phi_p; high];
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
    u = -K_disc * xobs + G_disc * ref;   
    
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
    
    % Sistema lineal en tiempo discreto
    x = Ad * x + Bd * uu;
    
    % Observador en tiempo discreto
    y = Cd * x;          % Salida real
    y_obs = Cd * xobs;   % Salida estimada por el observador
    e = y - y_obs;       % Error de estimación
    xobs = Ad * xobs + Bd * uu + Ko_disc * e; % Dinámica del observador
    
    % Se guardan las estimaciones del observador
    alpha_obs_hist(i) = xobs(1);
    phi_obs_hist(i) = xobs(2);
    phi_p_obs_hist(i) = xobs(3);
    high_obs_hist(i) = xobs(4);
    
    u_hist(i) = uu;
end

% Gráficos
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