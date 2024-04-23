clear all;close all;

%Item 6
%Se implementa un PID en tiempo discreto para que el ángulo del motor permanezca en una
%referencia de 1radian, con los valores obtenidos en el Item 5

%A = [-Ra/Laa -Km/Laa 0; Ki/J -B/J 0; 0 1 0];
%B = [1/Laa 0; 0 -1/J; 0 0];
%C = [1 0 0; 0 1 0; 0 0 1];
%D = [0 0; 0 0; 0 0]
%U = [12; 0]

%Constantes del PID

color_='b';
Kp=0.1;
Ki=0.04;
Kd=0.008;
%Kd = 5;    %se modifica para evitar el sobrepaso en la salida del sistema

X = -[0; 0; 0];
ii = 0;
Ts = 1e-7;
wRef = 2;
tF = 1e-3;

A1 = ((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1 = (-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1 = Kd/Ts;
e = zeros(tF/Ts,1);
u = 0;

for t=0:Ts:tF
    
    ii=ii+1;
    k=ii+2;
    X=modmotor(Ts, X, u);
    e(k)=wRef-X(3); %ERROR
    u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
    
    x1(ii)=X(1);%Omega
    x2(ii)=X(2);%wp
    theta(ii)=X(3);%tita
    
    acc(ii)=u;
end
t=0:Ts:tF;
subplot(2,1,1);hold on;
plot(t,x1,color_);title('Salida y, \omega_t');
subplot(2,1,2);hold on;
plot(t,theta,color_); title('Salida Controlada por PID');
xlabel('Tiempo [Seg.]');
