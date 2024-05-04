clear all; close all;

%Función de transferencia continua G(s) 
P = [-2 0];             % Polos
K = [5];                % Ganancia
Sp = 10;                 % Sobrepasamiento

G  = zpk([],P,K)      % Funcion de transferencia
Tm = 0.06;               % Tiempo de muestro
Gd = c2d(G,Tm,'zoh')     % FdT discreta
Gd1 = c2d(G,10*Tm,'zoh') % Multiplicar Tm*10


figure
subplot(2,2,1)
pzmap(G)                
title('Mapa de Polo/Ceros:Continua')
resaltarPZ();
subplot(2,2,2)
pzmap(Gd)
title('Mapa de Polo/Ceros:Discreta')
resaltarPZ();
subplot(2,2,3)
step(Gd)
title('Respuesta al escalon')

subplot(2,2,4)
pzmap(Gd1)
title('Función de transf. con Tm*10')
resaltarPZ();

figure
subplot(2,2,1)
step(G)
title ('Respuesta en Continua')
subplot(2,2,2)
step(Gd)
title('Respuesta en Discreto')
 
Kp = dcgain(Gd)         % Error de posicion Kp                      
F  = feedback(Gd,1)     % Funcion con relimentacion unitaria
ess = 1/(1+Kp)          % Error en estado estacionario

figure
subplot(2,2,1)
step(F)                 % Respuesta al escalon sist realimentado
title('Respuesta al escalon unitario')            

t = 0:Tm:100*Tm;        % Rampa
subplot(2,2,2)
lsim(F,t,t)             % Respuesta a rampa sist realimentado

%Calculo de Kv y error debido a rampa
Kv = dcgain(Gd)
ess1 = 1 / Kv

subplot(2,2,3)
rlocus(G)               % Lugar de raíces FdT continua
title('Lugar de Raíces de Continua')

subplot(2,2,4)
rlocus(Gd)              % Lugar de raíces FdT discreto
title('Lugar de Raíces de Discreta')

[Gm,Pm] = margin(Gd)    % Ganancia crítica y margen de fase

rlocus(Gd1)            % LDR del sistema discreto Tm*10
[Gm1,Pm1] = margin(Gd1)

figure
rlocus=(Gd1) 

function resaltarPZ()
    % Encontrar los objetos de línea que representan los polos y ceros
    h = findobj(gca,'Type','Line');
    % Cambiar el espesor para resaltar los polos y ceros
    set(h,'LineWidth',2)
end
