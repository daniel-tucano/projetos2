clear
clc
close all

%% Dados

load("curva_tracao");

velocidadeRotacao= 60;
anguloClearence = 10;
anguloStall = 13;
coeficienteDeAtrito = 0.04;
mtow = 32500;                           %Kg
g = 9.81;                               %m/s²
area = 76.90;                           %m²
peso = mtow.*g;                         %N
clMax = 1.6;
clMaxComFlap = 2.19951;
densidadeArNivelDoMar = 1.225;          %Kg/m³
densidadeAr = 1.225;
inerciaEmTornoDoTremDePouso = 10^6;     %Kg.m²
obstaculoFimDecolagem = 10.668;         %m

% A não ser que esteja especificado as distâncias são em relação ao bordo
% de ataque da asa

cg.x = 0.2;
cg.y = 0.1;

tremDePouso.x = -0.5;
bequilha.x = 2.5;
bracoPesoTremDePouso = cg.x - tremDePouso.x;
MomentoPesoTremDePouso = bracoPesoTremDePouso * peso;

empenagemHorizontal.x = -16.3;
empenagemHorizontal.area = 12.78;
empenagemHorizontal.distanciaTremDePouso = empenagemHorizontal.x - tremDePouso.x;
empenagemHorizontal.distanciaCgX = empenagemHorizontal.x - cg.x;
empenagemHorizontal.clAlpha = 0.0260;
empenagemHorizontal.cl0 = 0.0;
anguloComandoProfundor = 12;

empenagemVertical.y = 5.0;
empenagemVertical.area = 10.95;
empenagemVertical.distanciaCgY = empenagemVertical.y - cg.y;
empenagemVertical.clAlpha = 2*pi;
empenagemVertical.cl0 = 0.0;

A = readmatrix("pontos_polar_arrasto_nova_com_flap.xlsx");

asa.alpha = A(:,1)';
asa.cl = A(:,3)';
asa.cd = A(:,6)';
asa.cm = A(:,9)';


%% Simulação decolagem

dt = 0.01;
% simulando 2 minutos
nIntervalosDeTempo = 2*60/dt;
t = 0.0:dt:(nIntervalosDeTempo-1)*dt;
tArfagem = 0.0;

aceleracaoHorizontal = zeros(1,nIntervalosDeTempo);
velocidadeHorizontal = zeros(1,nIntervalosDeTempo);
deslocamentoHorizontal = zeros(1,nIntervalosDeTempo);

aceleracaoVertical = zeros(1,nIntervalosDeTempo);
velocidadeVertical = zeros(1,nIntervalosDeTempo);
deslocamentoVertical = zeros(1,nIntervalosDeTempo);

MomentoProfundor = zeros(1,nIntervalosDeTempo);
MomentoArfagem = zeros(1,nIntervalosDeTempo);
aceleracaoAngular = zeros(1,nIntervalosDeTempo);
velocidadeAngular = zeros(1,nIntervalosDeTempo);
anguloArfagemFuselagem = zeros(1,nIntervalosDeTempo);

anguloDeAtaqueAsa = zeros(1,nIntervalosDeTempo);
anguloDeAtaqueProfundor = zeros(1,nIntervalosDeTempo);

anguloVelocidade = zeros(1,nIntervalosDeTempo);

clAsa = zeros(1,nIntervalosDeTempo);
SustentacaoAsa = zeros(1,nIntervalosDeTempo);
Arrasto = zeros(1,nIntervalosDeTempo);
Tracao = zeros(1,nIntervalosDeTempo);
Normal = zeros(1,nIntervalosDeTempo);
Atrito = zeros(1,nIntervalosDeTempo);
SustentacaoProfundor = zeros(1,nIntervalosDeTempo);

ForcaHorizontal = zeros(1,nIntervalosDeTempo);
ForcaVertical = zeros(1,nIntervalosDeTempo);

determinouDistanciaDecolagem = false;
distanciaDecolagem = 0.0;

for iTempo = 1:(nIntervalosDeTempo-1)
    
    % Enquanto não for atingido o angulo de clearence rotacionamos a
    % aeronave
    if velocidadeHorizontal(iTempo) >= velocidadeRotacao && anguloArfagemFuselagem(iTempo) < anguloClearence
        
        anguloDeAtaqueProfundor(iTempo) = anguloComandoProfundor - anguloArfagemFuselagem(iTempo) - anguloVelocidade(iTempo);
        clProfundor = empenagemHorizontal.cl0 + empenagemHorizontal.clAlpha * anguloDeAtaqueProfundor(iTempo);
        SustentacaoProfundor(iTempo) = clProfundor * velocidadeHorizontal(iTempo)^2 * empenagemHorizontal.area * densidadeAr/2;
%         MomentoProfundor(iTempo) = empenagemHorizontal.distanciaCgX * SustentacaoProfundor(iTempo);
%         
%         MomentoArfagem(iTempo) = MomentoProfundor(iTempo) - MomentoPesoTremDePouso;
        aceleracaoAngular(iTempo) = 24/25*(5 - 2 * tArfagem);        
        velocidadeAngular(iTempo+1) = velocidadeAngular(iTempo) + aceleracaoAngular(iTempo)*dt;
        anguloArfagemFuselagem(iTempo+1) =  anguloArfagemFuselagem(iTempo) + velocidadeAngular(iTempo)*dt + aceleracaoAngular(iTempo)*dt^2/2;
       
        tArfagem = tArfagem + dt;
    elseif anguloArfagemFuselagem(iTempo) >= anguloClearence
        velocidadeAngular(iTempo+1) = 0.0;
        anguloArfagemFuselagem(iTempo+1) = anguloClearence;
        anguloDeAtaqueProfundor(iTempo) = anguloComandoProfundor - anguloArfagemFuselagem(iTempo) - anguloVelocidade(iTempo);
    end
    
    anguloDeAtaqueAsa(iTempo) = anguloArfagemFuselagem(iTempo) - anguloVelocidade(iTempo);
    iAnguloAtaque = indice_mais_proximo(asa.alpha, anguloDeAtaqueAsa(iTempo));
    clAsa(iTempo) = lerp(asa.alpha(iAnguloAtaque-1), asa.alpha(iAnguloAtaque), asa.cl(iAnguloAtaque-1), asa.cl(iAnguloAtaque), anguloDeAtaqueAsa(iTempo));
    SustentacaoAsa(iTempo) = clAsa(iTempo)*velocidadeHorizontal(iTempo)^2*densidadeAr*area/2;
    Tracao(iTempo) = curvaTracao.tracao(indice_mais_proximo(curvaTracao.velocidades, velocidadeHorizontal(iTempo)));
    cdAsa = lerp(asa.alpha(iAnguloAtaque-1), asa.alpha(iAnguloAtaque), asa.cd(iAnguloAtaque-1), asa.cd(iAnguloAtaque), anguloDeAtaqueAsa(iTempo));
    Arrasto(iTempo) = cdAsa*velocidadeHorizontal(iTempo)^2*densidadeAr*area/2;
    
    if SustentacaoAsa(iTempo) * cosd(anguloVelocidade(iTempo)) - SustentacaoProfundor(iTempo) + Tracao(iTempo) * sind(anguloArfagemFuselagem(iTempo)) - Arrasto(iTempo)*sind(anguloVelocidade(iTempo)) - peso <= 0.0 && deslocamentoVertical(iTempo) <= 0.01
        Normal(iTempo) = peso - SustentacaoAsa(iTempo) * cosd(anguloVelocidade(iTempo)) + SustentacaoProfundor(iTempo) - Tracao(iTempo) * sind(anguloArfagemFuselagem(iTempo)) + Arrasto(iTempo)*sind(anguloVelocidade(iTempo));
    else
        Normal(iTempo) = 0.0;
    end
   
    Atrito(iTempo) = Normal(iTempo) * coeficienteDeAtrito;
    
    ForcaHorizontal(iTempo) = Tracao(iTempo)*cosd(anguloArfagemFuselagem(iTempo)) - Arrasto(iTempo)*cosd(anguloVelocidade(iTempo)) - Atrito(iTempo) - SustentacaoAsa(iTempo)*sind(anguloVelocidade(iTempo));
    aceleracaoHorizontal(iTempo) = ForcaHorizontal(iTempo)/mtow;
    velocidadeHorizontal(iTempo+1) = velocidadeHorizontal(iTempo) + aceleracaoHorizontal(iTempo)*dt;
    deslocamentoHorizontal(iTempo+1) = deslocamentoHorizontal(iTempo) + velocidadeHorizontal(iTempo)*dt + aceleracaoHorizontal(iTempo)*dt^2/2;
    
    ForcaVertical(iTempo) = SustentacaoAsa(iTempo) * cosd(anguloVelocidade(iTempo)) - SustentacaoProfundor(iTempo) - peso + Normal(iTempo) - Arrasto(iTempo)*sind(anguloVelocidade(iTempo)) + Tracao(iTempo) * sind(anguloArfagemFuselagem(iTempo));
    aceleracaoVertical(iTempo) = ForcaVertical(iTempo)/mtow;
    velocidadeVertical(iTempo+1) = velocidadeVertical(iTempo) + aceleracaoVertical(iTempo)*dt;
    deslocamentoVertical(iTempo+1) = deslocamentoVertical(iTempo) + velocidadeVertical(iTempo)*dt + aceleracaoVertical(iTempo)*dt^2/2;

    anguloVelocidade(iTempo+1) = atand(velocidadeVertical(iTempo+1)/velocidadeHorizontal(iTempo+1));
    
    if ~determinouDistanciaDecolagem && deslocamentoVertical(iTempo) >= obstaculoFimDecolagem
        distanciaDecolagem = deslocamentoHorizontal(iTempo);
        fprintf("distancia de decolagem: %d \n", distanciaDecolagem)
        determinouDistanciaDecolagem = true;
    end
end

%% Plots

tPlot = 50;

% forças verticais x tempo
plot(t, SustentacaoAsa)
hold on
grid minor
plot(t, SustentacaoProfundor)
plot(t, Normal)
plot(t, ForcaVertical)
xlabel("Tempo [s]")
ylabel("Força [N]")
legend("Sustentação Asa", "Sustentação Profundor", "Normal", "Força Vertical")
title("Forças Verticais x tempo")
axis([0 tPlot -1*10^5 5*10^5])

% forças horizontais
figure
plot(t, Tracao)
hold on
grid minor
plot(t, Arrasto)
plot(t, Atrito)
plot(t, ForcaHorizontal)
xlabel("Tempo [s]")
ylabel("Força [N]")
legend("Tração", "Arrasto", "Atrito", "Força Horizontal")
title("Forças Horizontais x tempo")
axis([0 tPlot 0 150000])

% momento arfagem
% figure
% plot(t, MomentoProfundor)
% hold on
% grid minor
% plot(t, ones(1,nIntervalosDeTempo) * MomentoPesoTremDePouso)
% plot(t, MomentoArfagem)
% legend("Momento Profundor", "Momento Peso", "Momento de Arfagem Total")
% title("Momentos de Arfagem em torno do trem de pouso x tempo")

% aspectos cinematicos horizontal
figure
plot(t, aceleracaoHorizontal)
hold on
grid minor
plot(t, velocidadeHorizontal)
% plot(t, deslocamentoHorizontal)
xlabel("Tempo [s]")
ylabel("Velocidade [m/s] aceleração [m/s²]")
legend("Aceleração Horizontal", "Velocidade Horizontal")
title("Aspectos cinemáticos na horizontal")
axis([0 tPlot 0 100])

% aspectos cinematicos vertical
figure
plot(t, aceleracaoVertical)
hold on
grid minor
plot(t, velocidadeVertical)
% plot(t, deslocamentoHorizontal)
xlabel("Tempo [s]")
ylabel("Velocidade [m/s] aceleração [m/s²]")
legend("Aceleração Vertical", "Velocidade Vertical")
title("Aspectos cinemáticos na vertical")
axis([0 tPlot 0 20])

% aspectos cinematicos angulares
figure
% plot(t, aceleracaoAngular)
hold on
grid minor
% plot(t, velocidadeAngular)
plot(t, anguloDeAtaqueAsa)
plot(t, anguloDeAtaqueProfundor)
plot(t, anguloVelocidade)
xlabel("Tempo [s]")
ylabel("Velocidade angular [°/s] aceleração angular [°/s²]")
legend("Angulo De Ataque Asa", "Angulo de Ataque Profundor", "Angulo Velocidade")
title("Aspectos cinemáticos angulares de arfagem")
axis([0 tPlot 0 15])

% Cl asa x tempo
figure
plot(t, clAsa)
xlabel("Tempo [s]")
ylabel("Cl asa []")
grid minor
title("Cl da asa x tempo")

% deslocamento horizontal x vertical
figure
plot(deslocamentoHorizontal(t < tPlot), deslocamentoVertical(t < tPlot))
hold on
plot([1 1]*distanciaDecolagem, [-10^10, 10^10], "--")
xlabel("Deslocamento Horizontal [m]")
ylabel("Deslocamento Vertical [m]")
grid minor
title("deslocamento Vertical x delocamento Horizontal")
axis([0 1500, -700, 800])
legend("tragetoria", "fim da decolagem")