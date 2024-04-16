% ITEM 5------------------------------------------------------------------------
clear all; close all;

% Cargamos los datos medidos
valores = xlsread('Curvas_Medidas_Motor_2024.xls');
t = valores(:,1);
wr = valores(:,2);
ia = valores(:,3);
Vi = valores(:,4);
TL = valores(:,5);

figure('Name', 'Ítem 5')
 subplot(4,1,1);plot(t,ia);grid on; title('Corriente de Armadura, I');
 subplot(4,1,2);plot(t,wr);grid on; title('Velocidad Angular, Wr');
 subplot(4,1,3);plot(t,Vi);grid on; title('Tensión de entrada, Vi');
 subplot(4,1,4);plot(t,TL);grid on; title('Torque de carga, TL');
 
% Obtención de la funcion de transferencia del motor
syms V Ra La I Ki w Jm Km Bm Tl s real
eq1=s*I==-Ra/La*I-Km/La*w+1/La*V;
eq2=s*w==Ki/Jm*I-Bm/Jm*w;
S1=solve(eq1,eq2,w,V);
wr_va=collect(S1.w/S1.V,s);
pretty(wr_va)

eq1=s*I==-Ra/La*I-Km/La*w;
eq2=s*w==Ki/Jm*I-Bm/Jm*w-1/Jm*Tl;
S1=solve(eq1,eq2,w,Tl);
wr_Tl=collect(S1.w/S1.Tl,s);
pretty(wr_Tl)

% Determinamos las funciones de transferencia a partir del método desarollado
% por Chen

% Se definen los valores de tiempo y respuesta del sistema
t1 = 0.00005;   y1 = valores(703, 2);
t2 = 0.00005;   y2 = valores(704, 2);
t3 = 0.00005;   y3 = valores(705, 2);


% Se definen las k correspondientes a las 3 ecuaciones para los puntos tomados
% Ganancia seleccionada desde el gráfico, dividida por 12
K = max(wr)
k1 = (y1 / K-1) 
k2 = (y2 / K-1)
k3 = (y3 / K-1)

% Se calculan alfa 1, alfa 2 y beta
b = 4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3;
alfa1 = (k1*k2+k3-sqrt(b))/(2*(k1^2+k2));
alfa2 = (k1*k2+k3+sqrt(b))/(2*(k1^2+k2));
beta = (2*k1^3+3*k1*k2+k3-sqrt(b))/sqrt(b);

% Se calculan los tiempos de respuesta T1 y T2
T1 = -(t1) / log(alfa1)
T2 = -(t1) / log(alfa2)

% Se calcula la función de transferencia G

s = tf('s');
G = K / (T1 * s + 1) / (T2 * s + 1)/12

figure
hold on
step(G*12,5e-4, 'r')
plot(valores(:,1)-0.0351,valores(:,2));grid on; title('Velocidad Angular W_r');
grid on; 
title('Velocidad Angular Wr');
legend('FdT estimado','FdT medido')
%ylim([0, 2500]);
hold off


%Obtención de parámetros del motor
%Aproximación de la resistencia de armadura
Ra = max(Vi)/max(ia)
[N D] = tfdata(G, 'v')
Ki = N(3)
Km = Ki
Bm = (D(3)-(Ki*Km))/Ra