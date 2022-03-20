clear
clc
close all

%% Dados

potenciaDoMotor = 4095384;  %[Watts]
diametroHelice = 4.11; %[m]
rotacaoHelice = 1020/60; %[rev/s]
velocidades = 1:0.25:185;   %[m/s]
razoesDeAvanco = velocidades/(rotacaoHelice * diametroHelice);
pontosCurvaEficiencia = readmatrix("pontos_eficiencia_helice.csv");
polinominoCurvaEficiencia = polyfit(pontosCurvaEficiencia(:,1), pontosCurvaEficiencia(:,2),10);

xCurvaEfficiencia = 0:0.05:2.6;
yCurvaEfficiencia = polyval(polinominoCurvaEficiencia, razoesDeAvanco);

%% Cálculos

potenciaPropulsiva = potenciaDoMotor .* polyval(polinominoCurvaEficiencia, razoesDeAvanco);
tracao = potenciaPropulsiva ./ velocidades ;

%% Plots

plot(velocidades, tracao);
grid minor
xlabel("Velocidade [m/s]")
ylabel("Tração [N]")

figure
plot(razoesDeAvanco, yCurvaEfficiencia)
grid minor
axis([0, 3.6, 0, 1])
hold on
plot(pontosCurvaEficiencia(:,1), pontosCurvaEficiencia(:,2), 'o')