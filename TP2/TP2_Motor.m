%Se pide Implementar un sistema en variables de estado que controle el ángulo del motor, para
%consignas de pi/2 y –pi/2 cambiando cada 5 segundos y que el TL es el descripto en la planilla de datos
%comparando el desempeño con el obtenido con el PID digital del TP Nº1. Hallar el valor de
%integración Euler adecuado

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
Q = diag([1/(1.5)^2, 1/(200)^2, 1/(0.4)^2, 100000]) %penalización de ia, w, tita y el integrador del error   
%Q = diag([6.25, 2.5e-5, 0.4444, 100000])
R = 100;
Aamp = [A zeros(3,1); -C 0]
Bamp = [B; 0]
K = lqr(Aamp, Bamp, Q, R)

% Cálculo de los polos a lazo cerrado y tiempo de integración
poles = eig(Aamp - Bamp * K)
lambda = max(abs(real(poles)))  %parte real más rápida
tR = log(0.95) / lambda
h = 10e-7;                    
simTime = 5;                    % tiempo de simulación en segundos
t = 0:h:simTime;

%Señal de referencia y torque de entrada
reference = (pi/2) * square(2 * pi * (1/5) * t);   %alterna cada 5 segundos

figure
plot(t,reference,'LineWidth',1.5)
xlabel('Tiempo [seg]')
ylabel('Ángulo [rad]')
title('Señal de referencia \theta_i')
grid

torque = ((1.1e-3) / 2) * square(2 * pi * (1/5) * t) + ((1.1e-3) / 2);

figure
plot(t,torque,'LineWidth',1.5)
xlabel('Tiempo [seg]')
ylabel('Torque [Nm]')
title('Torque de entrada T_L')
grid

%Condiciones iniciales
ia(1) = 0;
w(1) = 0;
tita(1) = 0;
u = zeros(1, length(t));
psi = zeros(1, length(t));

%xop = [0 0 0]';
x = [ia(1) w(1) tita(1)];
stateVector = [ia(1) w(1) tita(1)]';  %vector de etados del sistema
integ = 0;


for i = 1:length(t)
    zetaP = reference(i)-C*stateVector;
    zeta(i) = integ+zetaP*h;
    u(i) = -K(1:3)*stateVector-K(4)*zeta(i);
    ia(i) = x(1);
    w(i) = x(2);
    tita(i) = x(3);
    x1P = -Ra*x(1)/Laa-Km*x(2)/Laa+u(i)/Laa;
    x2P = Ki*x(1)/J-Bm*x(2)/J-torque(i)/J;
    x3P = x(2);
    xP = [x1P x2P x3P]';
    x = x+h*xP;
    stateVector = [ia(i) w(i) tita(i)]';
    integ = zeta(i);
    
    %se evita que se superen los 24V
    if u(i) > 24
        u(i) = 24;
    elseif u(i) < -24
        u(i) = -24;
    end
    
    
end
save('iaReal.mat','ia');

figure;
subplot(2, 2, 1);
plot(t, ia, 'LineWidth', 1.5);
title('Corriente de armadura i_a');
xlabel('Tiempo (seg.)');
ylabel('Corriente (A)');
grid on;

subplot(2, 2, 2);
plot(t, tita, 'LineWidth', 1.5); hold on;
plot(t, reference, '--', 'LineWidth', 1.5); hold off;
title('Posición angular \theta_t');
xlabel('Tiempo (seg.)');
ylabel('Posición angular (Rad)');
legend('Salida', 'Referencia');
grid on;

subplot(2, 2, 3);
plot(t, u, 'LineWidth', 1.5);
title('Acción de control u_t');
xlabel('Tiempo (seg.)');
ylabel('Voltaje (V)');
grid on;
