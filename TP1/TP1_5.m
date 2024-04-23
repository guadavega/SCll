clc, clear all, close all;
%Item 5

%Se obtiene el modelo del sistema considerando como entrada un escalón de 
%12V, como salida a la velocidad angular y un torque de carga TL

% Cargamos los datos medidos
valores = xlsread('Curvas_Medidas_Motor_2024.xls');
t = valores(:,1);
w_r = valores(:,2);
i_a = valores(:,3);
Vi = valores(:,4);
T_L = valores(:,5);

figure('Name', 'Ítem 5')
 subplot(4,1,1);plot(t,i_a);grid on; title('Corriente de Armadura, I');
 subplot(4,1,2);plot(t,w_r);grid on; title('Velocidad Angular, Wr');
 subplot(4,1,3);plot(t,Vi);grid on; title('Tensión de entrada, Vi');
 subplot(4,1,4);plot(t,T_L);grid on; title('Torque de carga, TL');
 
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
t1 = 0.00005;            %inicio del tiempo de medidas
y1 = valores(703, 2);
y2 = valores(704, 2);
y3 = valores(705, 2);


% Se definen las k correspondientes a las 3 ecuaciones para los puntos tomados
% Ganancia seleccionada desde el gráfico, dividida por 12
K = max(w_r)
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
plot(t-0.0351,w_r);grid on; title('Velocidad Angular W_r');
grid on; 
title('Velocidad Angular Wr');
legend('FdT estimado','FdT medido')
hold off


%Obtención de parámetros del motor
%Aproximación de la resistencia de armadura
%Ra = max(Vi)/max(i_a)
Ra = 20
Km = max(Vi)/max(w_r)
K_ss = (198-164)/1.05e-3
Ki = Ra/Km/K_ss
[N D] = tfdata(G, 'v')

%Tomando el 3er coeficiente por ser el término de mayor grado en el 
%numerador de la función de transferencia, se normalizan N y D para 
%obtener la inercia e inducción del motor

norm = Ki/N(3)
N = N*norm
D = D*norm

B = 0;
J = D(2)/Ra
Laa = D(1)/J 

%Entrada de tensión para el motor

delta_T = 10e-7;
ts = 0.6;
t_s =0:delta_T:(ts-(delta_T));

%Integración por euler para obtener la matriz de las variables de estado 

ia = zeros(1,ts/delta_T);
wr = zeros(1,ts/delta_T);
theta = zeros(1,ts/delta_T);
va = zeros(1,ts/delta_T);
TL = zeros(1,ts/delta_T);

ia(1)=0;             %Condiciones iniciales para las 3 variables de estado
wr(1)=0; 
theta(1)=0; 

for i=1:(ts/delta_T)
    if i*delta_T < 0.0351    %a partir del tiempo 0.0351 la tensión es 12v
        va(i) = 0;
    else
        va(i) = 12;
    end
end

% si el punto de tiempo actual cumple con alguna de las cond. para que 
%el torque sea distinto de cero: 

for i = 1:(ts/delta_T)
     if i*delta_T < 0.1863       %en t=0.1863 el torque es cero según los datos del excel y luego varia
         TL(i) = 0;
     elseif i*delta_T < 0.3372   %en t=0.3372 el torque vuelve a ser cero
         TL(i) = 1.1e-3;
     elseif i*delta_T < 0.4866   %en t=0.4866 el torque vuelve a ser 0.0010
         TL(i) = 0;
     else
         TL(i) = 1.1e-3;
     end
end


for i = 2:(ts/delta_T-1)
    ia(i)=ia(i-1)+delta_T*(-Ra*ia(i-1)/Laa-Km*wr(i-1)/Laa+va(i-1)/Laa);
    wr(i)=wr(i-1)+delta_T*(Ki/J*ia(i-1)-B/J*wr(i-1)-TL(i-1)/J);
    theta(i) = theta(i-1) + delta_T*wr(i-1);
end

%disp(length(t))
%disp(length(t_s));
%disp(length(wr));
%disp(length(ia));
%disp(length(TL));

figure('Name','Torque vs Corriente')
 subplot(3,1,1);
 plot(t_s,wr);grid on; title('Velocidad angular, Wr'); 
 hold on
 ylim([0, 200]);
 plot(t,w_r);grid on;
 legend('Aprox', 'Medido')
 hold off
 
 subplot(3,1,2);plot(t_s,ia);grid on; title('Corriente de armadura, Ia');
 hold on
 ylim([0, 0.4]);
 plot(t,i_a);grid on;
 hold off
 
 
 subplot(3,1,3);plot(t_s,TL);grid on; title('Torque de carga, TL');
 hold on
 plot(t,T_L);
 hold off