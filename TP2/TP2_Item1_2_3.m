clc; clear all; close all;

% Parámetros del motor
Ra = 20;
Laa = 4.9736e-04;
Km = 0.0605;
J = 2.5518e-09;
Bm = 0;
Ki = 0.0102;

% Matrices de estado
A = [-Ra/Laa -Km/Laa 0; Ki/J -Bm/J 0; 0 1 0];
B = [1/Laa; 0; 0];
C = [0 0 1];
D = [0];

% Sistema de espacio de estados
sys = ss(A, B, C, D);

% Diseño del controlador LQR
Q = diag([1/(1.5)^2, 1/(200)^2, 1/(0.4)^2, 100000]); %penalización de ia, w, tita y el integrador del error   
R = 100;
Aamp = [A zeros(3,1); -C 0];
Bamp = [B; 0];
K = lqr(Aamp, Bamp, Q, R);

% Cálculo de los polos a lazo cerrado y tiempo de integración
poles = eig(Aamp - Bamp * K);
lambda = max(abs(real(poles)));  % Parte real más rápida
h = 1e-7;  % Tiempo de integración de Euler
simTime = 10;  % Tiempo de simulación en segundos
t = 0:h:simTime;

% Señal de referencia y torque de entrada
reference = (pi/2) * square(2 * pi * (1/5) * t);  % Alterna cada 5 segundos
torque = ((1.1e-3) / 2) * square(2 * pi * (1/5) * t) + ((1.1e-3) / 2);

% Condiciones iniciales
ia = zeros(1, length(t));
w = zeros(1, length(t));
tita = zeros(1, length(t));
u = zeros(1, length(t));
psi = zeros(1, length(t));

% Vector de estados del sistema
stateVector = [ia(1) w(1) tita(1)]';
integ = 0;

% Simulación del sistema
for i = 1:length(t)
    zetaP = reference(i) - C * stateVector;
    zeta(i) = integ + zetaP * h;
    u(i) = -K(1:3) * stateVector - K(4) * zeta(i);
    
    % Evitar que se superen los 24V
    if u(i) > 24
        u(i) = 24;
    elseif u(i) < -24
        u(i) = -24;
    end
    
    % Ecuaciones del sistema
    x1P = -Ra * stateVector(1) / Laa - Km * stateVector(2) / Laa + u(i) / Laa;
    x2P = Ki * stateVector(1) / J - Bm * stateVector(2) / J - torque(i) / J;
    x3P = stateVector(2);
    xP = [x1P; x2P; x3P];
    stateVector = stateVector + h * xP;
    
    ia(i) = stateVector(1);
    w(i) = stateVector(2);
    tita(i) = stateVector(3);
    integ = zeta(i);
end

save('iaReal.mat', 'ia');

% Graficar resultados
figure;
subplot(3, 1, 1);
plot(t, ia, 'LineWidth', 1.5);
xlabel('Tiempo [seg]');
ylabel('Corriente [A]');
title('Corriente de armadura i_a');
grid on;

subplot(3, 1, 2);
plot(t, tita, 'LineWidth', 1.5);
hold on;
plot(t, reference, '--', 'LineWidth', 1.5);
xlabel('Tiempo [seg]');
ylabel('Posición angular (Rad)');
title('Posición angular \theta_t');
legend('Salida', 'Referencia');
grid on;
hold off;

subplot(3, 1, 3);
plot(t, u, 'LineWidth', 1.5);
title('Acción de control u_t');
xlabel('Tiempo [seg]');
ylabel('Voltaje (V)');
grid on;

% Observador de estado
Ao = A';
Bo = C';
Co = B';
Qo = diag([100 10 1]);
Ro = 1;
Ko = lqr(Ao, Bo, Qo, Ro)';

obsStateVector = [0; 0; 0];
xObs = [0; 0; 0];

% Simulación del sistema con observador
iaO = zeros(1, length(t));
wO = zeros(1, length(t));
titaO = zeros(1, length(t));
zeta = 0;

for i = 1:length(t)
    zetaP = reference(i) - C * obsStateVector;
    zeta = zeta + zetaP * h;
    u(i) = -K(1:3) * obsStateVector - K(4) * zeta;
    
    % Evitar que se superen los 24V
    if u(i) > 24
        u(i) = 24;
    elseif u(i) < -24
        u(i) = -24;
    end
    
    % Actualización del observador
    y = C * stateVector;
    yO = C * xObs;
    xObs = xObs + h * (A * xObs + B * u(i) + Ko * (y - yO));
    
    % Estimación de estados
    iaO(i) = xObs(1);
    wO(i) = xObs(2);
    titaO(i) = xObs(3);
    
    % Ecuaciones del sistema
    x1P = -Ra * obsStateVector(1) / Laa - Km * obsStateVector(2) / Laa + u(i) / Laa;
    x2P = Ki * obsStateVector(1) / J - Bm * obsStateVector(2) / J - torque(i) / J;
    x3P = obsStateVector(2);
    xP = [x1P; x2P; x3P];
    obsStateVector = obsStateVector + h * xP;
end

%La corriente observada queda:
realIa = load('iaReal.mat');

figure;
subplot(2, 2, 1);
plot(t, iaO, 'LineWidth', 1.5);
xlabel('Tiempo [seg]');
ylabel('Corriente [A]');
title('Corriente de armadura i_a observada');
grid on;

subplot(2, 2, 2);
plot(t, iaO, 'LineWidth', 1.5);
hold on;
plot(t, realIa.ia, 'LineWidth', 1.5);
xlabel('Tiempo [seg]');
ylabel('Corriente [A]');
title('Corriente de armadura i_a');
legend('Observada', 'Real');
grid on;
hold off;

subplot(2, 2, 3);
plot(t, tita, 'LineWidth', 1.5);
hold on;
plot(t, reference, '--', 'LineWidth', 1.5);
xlabel('Tiempo [seg]');
ylabel('Posición angular (Rad)');
title('Posición angular \theta_t');
legend('Salida', 'Referencia');
grid on;
hold off;

subplot(2, 2, 4);
plot(t, u, 'LineWidth', 1.5);
title('Acción de control u_t');
xlabel('Tiempo [seg]');
ylabel('Voltaje (V)');
grid on;
