clc, clear all, close all;
%Item 6

%Se implementa un PID en tiempo discreto para que el ángulo del motor 
%permanezca en una referencia de 1 radian

color_='b';
X = -[0; 0];                   %definimos condiciones iniciales
ii = 0;
tS = 1e-7;
tF = 0.01; 
wRef = 2;
 
                                    %constantes del PID
Kp=0.1;
Ki=0.01;
Kd=5;                             
                          
A1=((2*Kp*tS)+(Ki*(tS^2))+(2*Kd))/(2*tS);
B1=(-2*Kp*tS+Ki*(tS^2)-4*Kd)/(2*tS);
C1=Kd/tS;
e=zeros(tF/tS,1);
u=0;

                                      %parámetros del motor
Laa = 366e-6;
J = 5e-9;
Ra = 55.6;
B = 0;
Ki = 6.49e-3;
Km = 6.53e-3;

%Predefinir vectores para almacenar resultados
x1 = zeros(1, length(0:tS:tF));         %Vector para almacenar la salida del sistema
acc = zeros(1, length(0:tS:tF));        %Vector para almacenar la entrada del sistema

for t=0:tS:tF
    ii=ii+1;k=ii+2;
    Va=u;                                    %tensión de entrada al motor
    
    h=1e-7;
    w= X(1);
    wp= X(2);
    
    for ii=1:(tS/h)
        wpp =(-wp*(Ra*J+Laa*B)-w*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa);
        wp = wp+h*wpp;
        w = w + h*wp;
    end
    
    X = [w; wp];                            %actualización del estado del motor
    x1(ii)=X(1);                            %Omega (w)
    x2(ii)=X(2);                            %wp

    e(k)=wRef-X(1);                         %error
    u=u+A1*e(k)+B1*e(k-1)+C1*e(k-2);        %ecuación del PID
    acc(ii)=u;
end

t=0:tS:tF;
subplot(2,1,1);
plot(t,x1,color_);title('Salida y, \omega_t');
subplot(2,1,2);
hold on;
plot(t,acc,color_);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');

