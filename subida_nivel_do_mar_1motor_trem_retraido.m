clear
clc
close all

%% Dados

% Polar de arrasto obtida no dia 14/03/2022 usando altitude de cruzeiro (FL250 - 7620m)

load("curva_tracao");
mtow = 32500;                           %Kg
g = 9.81;                               %m/s²
area = 76.90;                           %m²
peso = mtow.*g;                         %N
clMax = 1.6;
densidadeArNivelDoMar = 1.225;          %Kg/m³
densidadeAr = 1.225;
velocidades = curvaTracao.velocidades;  %m/s
tracaoDisponivel = curvaTracao.tracao * (densidadeAr ./ densidadeArNivelDoMar).^0.7./2; %N
A = readmatrix("pontos_polar_arrasto_nova_2.xlsx");

alpha = A(:,1)';
Cl = A(:,3)';
Cd = A(:,6)';
polinomioPolarDeArrasto = polyfit(Cl,Cd,5);

%% Calculos

velocidadeStall = sqrt(peso.*2./(area.*densidadeAr.*clMax));
ClRequerido = mtow.*g./(1/2.*velocidades.^2.*area.*densidadeAr);
CdRequerido = polyval(polinomioPolarDeArrasto, ClRequerido);
tracaoRequerida = 1/2.*densidadeAr.*CdRequerido.*velocidades.^2.*area;

excessoDePotencia = (tracaoDisponivel - tracaoRequerida) .* velocidades;
razaoDeSubida = excessoDePotencia./peso;
[maxRazaoDeSubida, iMaxRazaoDeSubida] = max(razaoDeSubida);

%% Encontrando velocidade minima e máxima propulsiva

velocidadesMenoresQueAMaximaRazaoDeSubida = velocidades(1:length(razaoDeSubida) <= iMaxRazaoDeSubida);
velocidadesMaioresQueAMaximaRazaoDeSubida = velocidades(1:length(razaoDeSubida) >= iMaxRazaoDeSubida);

razoesDeSubidaAntesDaMaximaRazaoDeSubida = razaoDeSubida(1:length(razaoDeSubida) <= iMaxRazaoDeSubida);
razoesDeSubidaDepoisDaMaximaRazaoDeSubida = razaoDeSubida(1:length(razaoDeSubida) >= iMaxRazaoDeSubida);

[~, iVelocidadeMinimaPropulsiva] = min(abs(razoesDeSubidaAntesDaMaximaRazaoDeSubida));
[~, iVelocidadeMaximaPropulsiva] = min(abs(razoesDeSubidaDepoisDaMaximaRazaoDeSubida));

velocidadeMinimaPropulsiva = velocidadesMenoresQueAMaximaRazaoDeSubida(iVelocidadeMinimaPropulsiva);
velocidadeMaximaPropulsiva = velocidadesMaioresQueAMaximaRazaoDeSubida(iVelocidadeMaximaPropulsiva);


%% Plots

% %Cl x alpha
% plot(alpha, Cl)
% grid minor
% xlabel("alpha [°]")
% ylabel("Cl")
%
% %Cd x alpha
% figure
% plot(alpha, Cd)
% grid minor
% xlabel("alpha [°]")
% ylabel("Cd")
%
%Cd x Cl
% figure
% plot(Cl,Cd,'o')
% hold on
% plot(Cl, polyval(polinomioPolarDeArrasto,linspace(0.037045,1.68,length(Cl))))
% grid minor
% xlabel("Cl")
% ylabel("Cd")
% legend("dados", "polinomio")

% Tração requerida e disponivel por velocidade
figure
plot(velocidades, tracaoRequerida, 'r')
hold on
plot(velocidades, tracaoDisponivel, 'b')
plot(ones(2).* velocidadeStall, [-10000, 150000],'-.g')
xlabel("Velocidade [m/s]")
ylabel("Tracao [N]")
legend("Tração Requerida", "Tração Disponivel", "Velocidade de Stall")
grid minor
axis([0 180 0 140000])

figure
plot(velocidades, tracaoDisponivel - tracaoRequerida)
xlabel("Velocidade [m/s]")
grid minor
axis([0 180 0 140000])

% Razão de subida 100% throttle
figure
plot(velocidades, razaoDeSubida./velocidades .* 100)
hold on
plot(ones(2).* velocidadeStall, [-10000, 150000],'-.g')
plot(ones(2).* 1.4 * velocidadeStall, [-10000, 150000],'-.r')
grid minor
xlabel("Velocidade horizontal [m/s]")
ylabel("razão de subida [%]")
axis([50 180 0 6])