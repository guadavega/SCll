clear all; close all; clc;

% Caso 1, punto 1

vin = 12;                     
R = 47;
L = 1e-6;
C = 100e-9;
                

A  = [ -R/L , -1/L ; 1/C , 0 ];  % matriz de estados
B  = [ 1/L ; 0 ];                % matriz de entrada
C1 = [ R , 0 ] ;                 % matrices de salidas
C2 = [ 0, R ];
D  = [ 0 ];                      % matriz de tranmision directa

[num,den] = ss2tf(A,B,C1,D);  % obtengo la funcion de transferencia a 
                              % partir de las matrices
G = tf(num,den)
polos = pole(G)               % obtengo los polos de la FT

tR = log(0.95)/polos(1);      % a partir de la dinámica mas rápida obtengo el tiempo de muestreo.
tR = tR*10                 
tR = 1e-9;                    % se redondea el resultado para simplificar

tS = log(0.05)/polos(2)       % a partir de la dinámica mas lenta obtengo el tiempo de simulación
tS = tS*150                    
%tS = 0.004;                  % se llega a ese valor para ver 2 periodos de
                              %trabajo
pasos = round(tS/tR)
retardo = 10000;              % Retardo de 1000 pasos 
t = 0:tR:tS;                  % vector de tiempo y de entrada  
u = linspace(0,0,pasos);

                              % Condiciones iniciales
                              
iL(1)  = 0;                   % Corriente por inductor
vC(1) = 0;                    % Tension en el capacitor
x      = [ iL(1) vC(1)]';      % Vector de estado 
y(1)   = 0;                   % Salida del sistema
Xop    = [0 0]';               % Punto de operación
tsim   = 0;                   % Contador de tiempo auxiLiar
                            
                              % Implementación de "for"
for i=1:pasos-1               % de "pasos-1" iteraciones

 tsim = tsim + tR;            
 if (tsim>=1e-4)              
    tsim=0;                   % cada 1mS la señal de entrada
    vin=vin*-1;               % pasa de 12V a -12V
 end                        
 if i >= retardo
        u(i+1) = vin;         %señal u con retardo
 else 
     u(i+1)=0;
 end                
 
 Xp = A*(x-Xop) + B*u(i);   % Xprima = A*x + B*u, en la iteracion "i"
 x = x + Xp*tR;             % Es una aproximación de la integración, se actualizan las variables de estado.
 
 Y = C1*x;                  % Se calcula la salida y = C*x
 
 y(i+1) = Y(1);             % Se guarda el valor de la salida
 iL(i+1) = x(1);            % Se guarda el valor de la corriente de inductor. x1 = iL
 vC(i+1) = x(2);            % Se guarda el valor de la tension del capacitor. x2 = vC
 
end

%Graficos de Il, Vc y Vin
figure(1)
subplot(3,1,1);
plot(t,iL, 'b' );title('Corriente , i_t'); grid on; 
ylim([-0.6 0.6]);
xlim([0 0.0005]);
subplot(3,1,2);
plot(t,vC, 'r' );title('Tensión Capacitor , Vc_t');grid on
ylim([-16 16]);
xlim([0 0.0005]);
subplot(3,1,3); 
plot(t,u, 'm' );title('Tensión de Entrada, u_t');grid on
ylim([-16 16]);
xlim([0 0.0005]);
