close all; clear all; clc;

m_nom = 1;          % masa nominal
m = m_nom;
color ='r';

% Para analizar robustez

%m = m_nom*0.90;
%color ='m'; 
%m = m_nom*1.1;
%color ='k'; 

b = 0.3;            %coeficiente de rozamiento
l = 1;              %longitud
g = 10;             %constante gravitatoria
delta = 90;         %ángulo de referencia en grados


% Linealizacion del sistema
Aa = [0 1; (-g/l)*cosd(delta) -b/(m/l^2)]
Bb = [0;1/(m*l^2)]
Cc = [1 0]

% Autovalores
autoval=eig(Aa)

% Comparacion de resultados
[A,B,C,D]=linmod('pendulo_mod_tarea',delta*pi/180)
autoval_2 = eig(A)

%Se verifica la controlabilidad de la matriz A
if (length(A)==rank(ctrb(A,B)))
    
    disp('La matriz A es controlable')

else
    
    disp('La matriz A NO es controlable')
    
end 


% Matrices ampliadas 
Aamp=[[A;C] zeros(3,1)]
Bamp=[B;0]
autoval_Aamp = eig(Aamp)   %autovalores, estabilidad y controlabilidad del sistema ampliado
rank(ctrb(Aamp,Bamp))

%Se verifica la controlabilidad de la matriz ampliada Aamp
if (length(Aamp)==rank(ctrb(Aamp,Bamp)))
    
    disp('La matriz Aamp es controlable')

else
    
    disp('La matriz Aamp NO es controlable')
    
end 

% Diseño de controlador por Asignación de Polos

p = -3;                 %polo triple
K = acker(Aamp,Bamp,[p p p])
k1 = K(1)
k2 = K(2)
k3 = K(3)
eig(Aamp-Bamp*K)        %polos lazo cerrado
tscalc = 7.5/(-p)       %tiempo de respuesta calculado

%Simulacion para PID
sim('pendulo_pid_tarea');
figure(1); 
plot(tout,yout,color,'LineWidth',2); grid on; title('Salida');hold on;
legend('m=1','m=0.9','m=1.1');legend('boxoff');


%Plano de fase
figure(2);
plot(yout,velocidad,color,'LineWidth',2); grid on; title('Plano de fases'); hold on;
legend('m=1','m=0.9','m=1.1');legend('boxoff');
%Torque total
figure(3); 
plot(tout,torque,color,'LineWidth',2); grid on;title('Torque');hold on;
legend('m=1','m=0.9','m=1.1');legend('boxoff');
%Accion integral
figure(4);
plot(tout,-accint,color,'LineWidth',2); grid on;title('Accion integral');hold on;
legend('m=1','m=0.9','m=1.1');legend('boxoff');

disp('Maximo valor de salida:')
ymax=max(yout)

disp('Sobrepaso en %:')
S=(ymax-delta)/delta*100

disp('Error relativo:')
erel=(delta-yout)/delta;

disp('Error final, debe ser cero:')
efinal=erel(end)

disp('Indice elementos con error relativo absoluto menor a 2:')
ind=find(abs(erel)>.02);

disp('Tiempo de establecimiento (ultimo valor del vector):')
tss=tout(ind(end))      

disp('Salida al tiempo ts:')
yte=yout(ind(end))

disp('Torque final:')
uf=torque(end)

disp('Accion integral final:')
Intf=-accint(end)          
