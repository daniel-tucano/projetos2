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
velocidades = curvaTracao.velocidades;  %m/s
A = readmatrix("pontos_polar_arrasto_nova_2.xlsx");

alpha = A(:,1)';
Cl = A(:,3)';
Cd = A(:,6)';
polinomioPolarDeArrasto = polyfit(Cl,Cd,5);


%% Encontrando velocidade maxima e minima para cada altitude densidade

altitudes = 0:50:15000;
velocidadesMinimasPorAltitude = zeros(1,length(altitudes));
velocidadesMaximasPorAltitude = zeros(1,length(altitudes));

for iAltitude = 1:length(altitudes)
    
    altitude = altitudes(iAltitude);
    [~,~,~, densidadeAr] = atmosisa(altitude);
    
    tracaoDisponivel = curvaTracao.tracao * (densidadeAr ./ densidadeArNivelDoMar).^0.7; %N
    
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
    
    if (velocidadeStall > velocidadeMinimaPropulsiva)
       velocidadesMinimasPorAltitude(iAltitude) = velocidadeStall; 
    else
        velocidadesMinimasPorAltitude(iAltitude) = velocidadeMinimaPropulsiva; 
    end
    
    velocidadesMaximasPorAltitude(iAltitude) = velocidadeMaximaPropulsiva;
    
end

%% Plots

plot(velocidadesMinimasPorAltitude, altitudes)
hold on
plot(velocidadesMaximasPorAltitude, altitudes)
grid minor
xlabel("velocidade [m/s]")
ylabel("altitude [m]")
legend("velocidade minima", "velocidade maxima")
title("analise de teto de operação")