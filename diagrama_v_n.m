clc; clear all; close all;

%% data
Gross_Weight = [71650 32500] ;%[lb kg]
Wing_Area = [8277.5 76.9]; %[ft2 m2]
Wing_Aspect_Ratio = 12;
Sweep_max_tc_line = 0;
Clmax = 1.6;
Clmin = -1.616;
Cruise_Speed = [269.98 500/3.6]; %[keas m/s];
Cruise_Mach = Cruise_Speed(2)/340; 
% Diving Speed
Vd = 1.25*Cruise_Speed(2); %m/s
Dive_Speed = [1.9438*Vd Vd]; %[keas m/s];
LoadFactor_max = 2.1+24000/(Gross_Weight(2)+10000);
LoadFactor_min = -1;
c = 2.603;
%% Sea Level 

%Velocidade de estol @ n=1
V_s = sqrt((2*Gross_Weight(2)*9.806)/(1.225*Wing_Area(2)*Clmax)); %m/s
%Corner Speed
V_star = sqrt(LoadFactor_max)*V_s; %m/s
% Velocidade @ LoadFactor_min
V_nmin = sqrt((2*LoadFactor_min*Gross_Weight(2)*9.806)/(1.225*Wing_Area(2)*Clmin)); %m/s

%% Estimation of Lift curve slope
Beta = sqrt(1-(Cruise_Mach^2));
a = 2*pi*Wing_Aspect_Ratio/(2+sqrt(4+(Wing_Aspect_Ratio^2)*(Beta^2)*(1+(tan(Sweep_max_tc_line))^2/Beta^2)));

% @ Cruise Speed
ug = 2*Gross_Weight(2)/(a*Wing_Area(2)*c);
Vg = 17.07; % m/s
Kg = 0.88*ug/(1+ug); %Fator de alivio de rajada
ng_c = [1-(0.5*a*Kg*Vg*Cruise_Speed(2)*1.225*Wing_Area(2)/(2*Gross_Weight(2)*9.806)) 1+(0.5*a*Kg*Vg*Cruise_Speed(2)*1.225*Wing_Area(2)/(2*Gross_Weight(2)*9.806))];

% @ Diving Speed
Vg = 17.07/2;
ng_d = [1-(0.5*a*Kg*Vg*Vd*1.225*Wing_Area(2)/(2*Gross_Weight(2)*9.806)) 1+(0.5*a*Kg*Vg*Vd*1.225*Wing_Area(2)/(2*Gross_Weight(2)*9.806))];

%% ENVELOPE

% stall curve
x_n = linspace(0,5,100);
Stall_Line_sup = sqrt(x_n)*V_s;
x_n2 = linspace(0,3,100);
Stall_Line_inf = sqrt(x_n)*V_nmin;
plot(Stall_Line_sup,x_n,'k','LineWidth',3);
hold on
plot(Stall_Line_inf,(-x_n2),'k','LineWidth',3);


% Superior Limit
x_nlim_sup = [V_star Vd];
Superior_limit = [LoadFactor_max LoadFactor_max];
hold on
plot(x_nlim_sup,Superior_limit,'k','LineWidth',3);
% Inferior Limit
x_nlim_inf = [84 Vd];
Inferior_limit = [LoadFactor_min LoadFactor_min];
hold on
plot(x_nlim_inf,Inferior_limit,'k','LineWidth',3);

% Structural Damage
x_supdam = [130 Vd];
x_infdam = [103 Vd];
Sup_Structural_Damage = 1.5*Superior_limit;
Inf_Structural_Damage = 1.5*Inferior_limit;
hold on
plot(x_supdam,Sup_Structural_Damage,'r','LineWidth',3);
hold on
plot(x_infdam,Inf_Structural_Damage,'r','LineWidth',3);
% Right - Structural Damage
x_rdam = [Vd Vd];
y_rdam = [Inf_Structural_Damage(1) Sup_Structural_Damage(1)];
hold on
plot(x_rdam,y_rdam,'r','LineWidth',3);

%-------------GUST----------------%
%Gust Enevolpe
x_ngc = [0:1:Cruise_Speed(2)];
x_ngd = [0:1:Vd];
y_ngc_pos = 1+ (ng_c(2)-1)*x_ngc/Cruise_Speed(2);
y_ngc_neg = 1+ (ng_c(1)-1)*x_ngc/Cruise_Speed(2);
y_ngd_pos = 1+ (ng_d(2)-1)*x_ngd/Vd;
y_ngd_neg = 1+ (ng_d(1)-1)*x_ngd/Vd;
% Limit Gust Envelope
x_limit_gust = [Cruise_Speed(2):1:Vd];
sup_limit_gust = (x_limit_gust-Vd)*(y_ngd_pos(end)-y_ngc_pos(end))/(Vd-Cruise_Speed(2)) + y_ngd_pos(end);
inf_limit_gust = (x_limit_gust-Vd)*(y_ngd_neg(end)-y_ngc_neg(end))/(Vd-Cruise_Speed(2)) + y_ngd_neg(end);
% Graph
hold on
plot(x_ngc,y_ngc_pos,'--g',x_ngc,y_ngc_neg,'--g',x_ngd,y_ngd_pos,'-.y',x_ngd,y_ngd_neg,'-.y',x_limit_gust,sup_limit_gust,'--g',x_limit_gust,inf_limit_gust,'--g','LineWidth',2);
ylim([-2 5]);
grid on;
title("V-n Diagram")
xlabel("Equivalent Air Speed [m/s]");
ylabel("n factor")


