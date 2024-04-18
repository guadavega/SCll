clear all;close all;

X = -[0; 0; 0];
ii = 0;
t_etapa = 1e-7;
wRef = 2;
tF = 1e-3;

Kp = 0.1;
Ki = 0.01;
%Kd = 5;
Kd = 2.1;       %se modifica para evitar el sobrepaso en la salida del sistema
color_='b';
Ts = t_etapa;
A1 = ((2*Kp*Ts)+(Ki*(Ts^2))+(2*Kd))/(2*Ts);
B1 = (-2*Kp*Ts+Ki*(Ts^2)-4*Kd)/(2*Ts);
C1 = Kd/Ts;
e = zeros(tF/t_etapa,1);
u = 0;

for t=0:t_etapa:tF
    
    ii=ii+1;
    k=ii+2;
    X=modmotor(t_etapa, X, u);
    e(k)=wRef-X(3); %ERROR
    u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2); %PID
    
    x1(ii)=X(1);%Omega
    x2(ii)=X(2);%wp
    theta(ii)=X(3);%tita
    
    acc(ii)=u;
end
t=0:t_etapa:tF;
subplot(2,1,1);hold on;
plot(t,x1,color_);title('Salida y, \omega_t');
subplot(2,1,2);hold on;
plot(t,theta,color_); title('Salida Controlada por PID');
xlabel('Tiempo [Seg.]');
