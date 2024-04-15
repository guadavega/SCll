clc, clear all, close all;
%Item 2
%Deducción de los valores R, L y C

valores = xlsread('Curvas_Medidas_RLC_2024.xls'); 
tt = valores(1:end,1);
I = valores(1:end,2);
Vc = valores(1:end,3);
muestra = 2001;

%se genera una señal que oscila entre +12 y -12 en un período de 0.10s
%se crea un vector llamado "u" de 2000 elementos y se inicializa con ceros
%se establece un valor para el tamaño del paso de tiempo
%se recorre el vector "u" utilizando un bucle for, comenzando en el índice 100,en cada iteración 
%se verifica si el índice actual es un múltiplo de 500. Si es así, se cambia el signo.

figure(1)
plot(tt,Vc, 'b' );title('Tension , V_t'); grid on;hold on; 
figure(2)
plot(tt,I, 'b' );title('Corriente , I_t');grid on;hold on;

t = linspace(0,0.2,muestra);           
u = linspace(0,0,muestra);              %definición del vector tiempo y entrada
vin = 12;                               %definición de tensión de entrada
ii = 0;

for i=1:muestra-1
 ii=ii+1;                               %ii representa la muestra actual
  if ii<100
      u(i)=0;
 elseif ii>=100 && ii<=500              %cada muestra es 5e-5
      u(i) = vin;
  elseif ii>500 && ii<1000 
      u(i) = vin*-1;
  elseif ii>1000 && ii<1500
      u(i) = vin;
  else 
      u(i) = vin * -1;
  end
end

figure
plot(t,u, 'm' );title('Tensión de Entrada, u_t');grid on

%utilizando el algoritmo de Chen, se hace la aproximación de FT para obtener R, L y C

%Se eligen los tres puntos según el artículo de Chen
t1_v=valores(126,1); y1_v=valores(126,3); 
t2_v=valores(151,1); y2_v=valores(151,3);
t3_v=valores(176,1); y3_v=valores(176,3);

figure(1)
plot([t1_v t2_v t3_v],[y1_v,y2_v,y3_v],'+');hold on;

t1_i=valores(103,1); y1_i=valores(103,2); 
t2_i=valores(105,1); y2_i=valores(105,2);
t3_i=valores(107,1); y3_i=valores(107,2);

figure(2)
plot([t1_i t2_i t3_i],[y1_i,y2_i,y3_i],'+');hold on;


%Ganancia seleccionada desde el grafico

K_v=valores(501,3)/12;     K_i=valores(501,2)/12; 

y1_v=y1_v/12; y2_v=y2_v/12; y3_v=y3_v/12;   %se dividen por 12 los puntos
y1_i=y1_i/12; y2_i=y2_i/12; y3_i=y3_i/12;   %seleccionados ya que se busca la rta
                                            % del sistema para el escalon unitario de entrada

%Defino las 3 k correspondientes a las 3 ecuaciones para los puntos tomados
k1_v= (y1_v/K_v) - 1;   k1_i= (y1_i/K_i) - 1;
k2_v= (y2_v/K_v) - 1;   k2_i= (y2_i/K_i) - 1;
k3_v= (y3_v/K_v) - 1;   k3_i= (y3_i/K_i) - 1;


%Despejo de las ecuaciones alfa 1, alfa 2 y beta
b_v=4*(k1_v^3)*k3_v-3*(k1_v^2)*(k2_v^2)-4*(k2_v^3)+(k3_v^2)+6*k1_v*k2_v*k3_v; %EC 23 Chen
alfa1_v=(k1_v*k2_v+k3_v-sqrt(b_v))/(2*(k1_v^2+k2_v)); %EC 21 Chen
alfa2_v=(k1_v*k2_v+k3_v+sqrt(b_v))/(2*(k1_v^2+k2_v)); %EC 22 Chen
beta_v=(k1_v+alfa2_v)/(alfa1_v-alfa2_v); %EC 20 Chen
%beta_v=(2*k1_v^3+3*k1_v*k2_v+k3_v-sqrt(b_v))/(sqrt(b_v)); %EC 24 Chen

b_i=(4*k1_i^3*k3_i)-(3*k1_i^2*k2_i^2)-(4*k2_i^3)+(k3_i^2)+(6*k1_i*k2_i*k3_i); %EC 23 Chen
alfa1_i=((k1_i*k2_i)+k3_i-sqrt(b_i))/(2*(k1_i^2+k2_i)); %EC 21 Chen
alfa2_i=((k1_i*k2_i)+k3_i+sqrt(b_i))/(2*(k1_i^2+k2_i)); %EC 22 Chen
beta_i=(k1_i+alfa2_i)/(alfa1_i-alfa2_i); %EC 20 Chen
%beta_i=(2*k1_i^3+3*k1_i*k2_i+k3_i-sqrt(b_i))/(sqrt(b_i)); %EC 24 Chen

%Sustituyendo EC 21 y EC 24 en EC 19 obtengo el cero y ambos polos
T1_v= -(t1_v-0.01)/log(alfa1_v);           T1_i= -(t1_i-0.01)/log(alfa1_i);
T2_v= -(t1_v-0.01)/log(alfa2_v);           T2_i= -(t1_i-0.01)/log(alfa2_i);
T3_v=(beta_v*(T1_v-T2_v))+T1_v;            T3_i=(beta_i*(T1_i-T2_i))+T1_i;


%Hago la nueva funcion de transferencia de tensión
G_v=tf(K_v*[T3_v 1],conv([T1_v 1],[T2_v 1]));

G_i=tf(K_i*[T3_i 1],conv([T1_i 1],[T2_i 1]));


%Se compara la Tensión y la corriente

[y_G_v,t_G_v]=lsim(G_v,u,t);
figure(4)
plot(tt,Vc, 'k' ); grid on; hold on;
plot(t_G_v,y_G_v,'r'); title('G_v obtenida con método de Chen vs Tensiónes de tabla');
legend({'V_t de excel','G_v obtenida con método de Chen'},'Location','southeast')

[y_G_i,t_G_i]=lsim(G_i,u,t);
figure(5)
plot(tt,I, 'k' ); grid on; hold on;
plot(t_G_i,y_G_i,'m'); title('G_i obtenida con método de Chen vs Corrientes de tabla');
legend({'I_t de excel','G_i obtenida con método de Chen'},'Location','southeast')

%De la función de transferencia sacamos
Cap=G_i.num{1}(2)
L=(G_i.den{1}(1))/Cap
R=(G_i.den{1}(2))/Cap


%Item 3
%Matrices
A=[-R/L -1/L; 1/Cap 0];
B=[1/L; 0];
C=[1 0];
D=0;
%Definicion de la ecuación de estado y de salida (salida de corriente)
G1=ss(A,B,C,D);
[yout,yt]=lsim(G1,(u),t);
figure(6)
plot(yt,yout,'b');grid on; hold on;
plot(valores(:,1),valores(:,2),'r'); title('Comparación de corriente');
legend({'i(t) aproximada con valores RLC calculados','i(t) de excel'},'Location','southeast')
