% ITEM 5------------------------------------------------------------------------
clear all; close all;
% Cargamos los datos medidos
datos = xlsread('Curvas_Medidas_Motor_2024.xlsx');
t = datos(:,1);
w_r= datos(:,2);
i_a = datos(:,3);
u = datos(:,4);
t_l = datos(:,5);

figure('Name', 'Ítem 5')
 subplot(4,1,1);plot(t,i_a);grid on; title('Corriente de Armadura, i_a');
 subplot(4,1,2);plot(t,w_r);grid on; title('Velocidad Angular, W_r');
 subplot(4,1,3);plot(t,u);grid on; title('Tensión de entrada, V_i');
 subplot(4,1,4);plot(t,t_l);grid on; title('Torque de carga, T_L');
 
% Obtención de la funcion de transferencia del motor
syms V Ra La I Ki w Jm Km Bm Tl s real
eq1=s*I==-Ra/La*I-Km/La*w+1/La*V;
eq2=s*w==Ki/Jm*I-Bm/Jm*w;
S1=solve(eq1,eq2,w,V);
wr_va=collect(S1.w/S1.V,s);
pretty(wr_va)

eq1=s*I==-Ra/La*I-Km/La*w;
eq2=s*w==Ki/Jm*I-Bm/Jm*w-1/j*Tl;
S1=solve(eq1,eq2,w,Tl);
wr_Tl=collect(S1.w/S1.Tl,s);
pretty(wr_Tl)

% Determinamos las funciones de transferencia a partir del método desarollado
% por Chen

% td: tiempo de delay de entrada
% t1: tiempo inicial para el algoritmo

td = 0.0351;
t1 = 0.00005;
k=max(w_r);
[val lugar] = min(abs((t1+td)-t))

k1 = w_r(lugar)/k-1
[val lugar] = min(abs((2*t1+td)-t))
k2 = w_r(lugar)/k-1
[val lugar] = min(abs((3*t1+td)-t))
k3 = w_r(lugar)/k-1

b=4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3;
alfa1 = (k1*k2+k3-sqrt(b))/(2*(k1^2+k2));
alfa2 = (k1*k2+k3+sqrt(b))/(2*(k1^2+k2));
beta= (2*k1^3+3*k1*k2+k3-sqrt(b))/sqrt(b);

T1 =-t1/log(alfa1);
T2 =-t1/log(alfa2);
%T3 = beta*(T1-T2)+T1
s = tf('s')
G1=k/(T1*s +1)/(T2*s +1)/12 %G1 normalizada.


figure
hold on
step(G1*12,5e-4, 'r')
plot(datos(:,1)-0.0351,datos(:,2));grid on; title('Velocidad Angular W_r');
legend('FdT estimado','FdT medido')
hold off







%Obtención de parámetros del motor
%Aproximación de la resistencia de armadura
Ra = max(u)/max(i_a)
[N D] = tfdata(G1, 'v')
Ki = N(3)
Km = Ki
Bm = (D(3)-(Ki*Km))/Ra





