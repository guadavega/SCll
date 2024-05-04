%clear all; close all;

%Función de transferencia continua G(s) 

P = [-2 0];             % Polos
K = [5];                % Ganancia
Sp = 10;                % Sobrepasamiento
tr = 2;                 % tiempo 2% de error

G  = zpk([],P,K)        % Funcion de transferencia
Tm = 0.06;              % Tiempo de muestro
Gd = c2d(G,Tm,'zoh')    % FdT discreta

%Obtención de W0;Wd;td;zita

zita=(-log(Sp/100))/sqrt(pi^2+log(Sp/100)^2)
W0=4/(tr*zita)
Wd=W0*sqrt(1-zita^2)
td=(2*pi)/Wd

%Obtención muestras por ciclo de la frec. amortiguada wd
m=td/Tm

%Ubicacion de los polos en el plano z
r=exp(-zita*W0*Tm)
ang=rad2deg(Wd*Tm)

rect=r*(cos(ang)+j*sin(ang))

%Diseño de controlador con sisotool
sisotool(Gd)

%Verificacion polos, ceros y respuesta al escalon temporal
C %muestra el compensador importado de sisotool 
F=feedback(C*Gd,1) % sistema de lazo cerrado
Pf = pole(F)
Cf = zero(F)
pzmap(F)
step(F) % respuesta al escalon
pid(C) %te da los valores de los K individuales
Pgd = pole(Gd)
Cgd = zero(Gd) 