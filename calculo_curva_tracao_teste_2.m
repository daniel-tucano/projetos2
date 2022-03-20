clear
clc
close all

%% Dados

quantidadeDeMotores = 2;
potenciaDoMotor = 4095384;  %[Watts]
diametroHelice = 4.11; %[m]
rotacaoHelice = 1020/60; %[rev/s]
velocidadesChute = 0:0.1:300;   %[m/s]
razoesDeAvanco = velocidadesChute/(rotacaoHelice * diametroHelice);
chuteEficiencia = 0.81;
velocidades = 0:0.1:300;



%% Cálculos

potenciaPropulsiva = potenciaDoMotor .* chuteEficiencia;

tracaoChute = potenciaPropulsiva ./ velocidadesChute ;
polinomioTracaoExtrapolacao = polyfit(velocidadesChute( velocidadesChute > 80), tracaoChute(velocidadesChute > 80), 2);
tracaoChute(tracaoChute > 65000) = 65000;

curvaTracao.velocidades = velocidades;
% Contabilizando dois motores
curvaTracao.tracao = quantidadeDeMotores * tracaoChute;

%% Salvando Respostas

save("curva_tracao.mat", 'curvaTracao');

%% Plots

plot(velocidades, polyval(polinomioTracaoExtrapolacao, velocidades));
hold on
plot(velocidadesChute, tracaoChute)
axis([0 180 0 10e4]);
grid minor
xlabel("Velocidade [m/s]")
ylabel("Tração [N]")

figure
plot(razoesDeAvanco, 100 * polyval(polinomioTracaoExtrapolacao, velocidades) .* velocidadesChute ./ potenciaDoMotor)
hold on
plot(razoesDeAvanco, 100 * tracaoChute .* velocidadesChute ./ potenciaDoMotor)
grid minor
ylabel("Eficiencia [%]")
xlabel("Razão de Avanço")