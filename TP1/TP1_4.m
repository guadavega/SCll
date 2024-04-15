clear all; close all;

%Item 4

Laa = 366e-6;
J = 5e-9;
Ra = 55.6;
B = 0;
Ki = 6.49e-3;
Km = 6.53e-3;
deltaT = 10e-07;
tF = 5;
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


for i=2:(tF/deltaT-1)
 ia(i)=ia(i-1)+deltaT*(-Ra*ia(i-1)/Laa-Km*w(i-1)/Laa+Va/Laa);
 w(i)=w(i-1)+deltaT*(Ki/J*ia(i-1)-B/J*w(i-1)-TL(i-1)/J);
 tita(i) = tita(i-1) + deltaT*w(i-1);
end

figure('Name','Torque vs Corriente')
 subplot(3,1,1);plot(t,w);grid on; title('Velocidad angular, W'); 
 subplot(3,1,2);plot(t,ia);grid on; title('Corriente de armadura, I_a');

                                 %El torque máximo viene dado por la 
 ia_max = max(ia)                %corriente máxima del motor (arranque)
 w_max = max(w)
 TL_max = Ki*ia_max-B*w_max
 
 figure('Name', 'W al aplicar TL máximo')
plot(t, w)
 