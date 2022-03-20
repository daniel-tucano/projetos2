clear
clc
close all

%% Dados

load("curva_tracao");
nMaxEstrutural = 2.67;
mtow = 32500;   %Kg
g = 9.81;       %m/s²
area = 76.90;   %m²
peso = mtow.*g; %N
clMax = 1.6;
densidadeArNivelDoMar = 1.225;          %Kg/m³
densidadeAr = 0.54939382277;            %Kg/m³
velocidades = curvaTracao.velocidades; %m/s
tracaoDisponivel = curvaTracao.tracao * (densidadeAr / densidadeArNivelDoMar).^0.7; %N
A = readmatrix("pontos_polar_arrasto_nova_2.xlsx");

alpha = A(:,1)';
Cl = A(:,3)';
Cd = A(:,6)';
polinomioPolarDeArrasto = polyfit(Cl,Cd,5);

%% Calculos

raioMinimoLimitanteEstrutural = velocidades.^2./(g.*sqrt(nMaxEstrutural-1));
nMaxAerodinamico = 1/2.*densidadeAr.*velocidades.^2.*clMax./(peso/area);
raioMinimoLimitanteAerodinamico = velocidades.^2./(g.*sqrt(nMaxAerodinamico-1));

%% Plots

plot(velocidades, raioMinimoLimitanteEstrutural)
hold on
plot(velocidades, raioMinimoLimitanteAerodinamico)
grid minor
xlabel("Velocidade [m/s]")
ylabel("Raio de Curvatura [m]")
axis([0 180 0 5000])
legend("Limitante Estrutural", "Limitante Aerodinamico")
title("Recuperação de mergulho - FL 250")