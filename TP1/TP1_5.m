clc, clear all, close all;

%Item 5
% Cargar las mediciones desde el excel
values = xlsread('Curvas_Medidas_Motor_2024.xls');
tt = values(1:end,1);           %Mediciones extraidas desde el excel
W = values(1:end,2);

deltaT = 10e-07;
tF = 0.5;
Va = 12;

t=0:deltaT:(tF-deltaT);         %vector tiempo 

for i=1:tF/deltaT-deltaT
 TL(i)=0;                       %inicializa el vector TL con todos sus elementos = 0
end
                                %Integración por euler para obtener la
ia = zeros(1,tF/deltaT);        %matriz de las variables de estado
w = zeros(1,tF/deltaT);
tita = zeros(1,tF/deltaT);
                                %Defino condiciones inicales nulas
ia(1)=0;
w(1)=0; 
tita(1)=0;      

figure(1)
subplot(2,1,1);hold on;
plot(tt,W, 'g' );title('Velocidad angular , \omega[rad/seg]'); grid on;hold on; 
plot(t,w,'r');hold on;
legend({'w de excel','w aproximada'},'Location','southeast')

subplot(2,1,2);hold on;
plot(t,Va,'r');title('Tension de Entrada');
xlabel('Tiempo [Seg.]');hold on;

for i=2:(tF/deltaT-1)
 ia(i)=ia(i-1)+deltaT*(-Ra*ia(i-1)/Laa-Km*w(i-1)/Laa+Va/Laa);
 w(i)=w(i-1)+deltaT*(Ki/J*ia(i-1)-B/J*w(i-1)-TL(i-1)/J);
 tita(i) = tita(i-1) + deltaT*w(i-1);
end

figure('Name','Torque vs Corriente')
 subplot(3,1,1);plot(t,w);grid on; title('Velocidad angular, W'); 
 subplot(3,1,2);plot(t,ia);grid on; title('Corriente de armadura, Ia');

                                 %El torque máximo viene dado por la 
 ia_max = max(ia)                %corriente máxima del motor (arranque)
 w_max = max(w)
 TL_max = Ki*ia_max-B*w_max