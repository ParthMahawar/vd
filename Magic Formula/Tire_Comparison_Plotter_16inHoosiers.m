%% Parameter loading
% 1: Hoosier 18.0x7.5-10 R25B, 8 in rim
% 2: Hoosier 16.0x7.5-10 R25B, 8 in rim

load('Fy_pure_parameters_run24_new.mat')
parameters{1} = Xbestcell;
load('Fy_pure_parameters_run6.mat')
parameters{2} = Xbestcell;

%% Plot settings
linS = {'-','-','--','--',':',':','-.','-.'};
label = {'Hoosier 18.0x7.5-10 R25B, 8 in rim',...
    'Hoosier 16.0x7.5-10 R25B, 8 in rim'};

%% Fy plotter

figure
for i = 1:2
    Xbestcell = parameters{i};
    alpha = linspace(-13,13,1000).';
    Fyplot = lateralforce_pure(Xbestcell,alpha,150,10,0);
    plot(alpha,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend
xlabel('Slip Angle (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Lateral Force Comparison (FZ = 250, P = 10, IA = 0)',...
    'FontSize',18);

P_input2 = [10];
IA_input2 = [0];
FZ_input2 = [150];
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2);
scatter(alpha2,Fy2,'DisplayName','18 in raw data');

%%
P_input2 = [10];
IA_input2 = [0];
FZ_input2 = [150];
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2);
scatter(alpha2,Fy2,'DisplayName','16 in raw data');

%% Peak Fy 

figure
for i = 1:2
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    Fyplot = -lateralforce_pure(Xbestcell,13,FZ,10,0);
    x{i} = Fyplot;
    plot(FZ,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Peak Lateral Force Comparison (P = 10, IA = 0)',...
    'FontSize',18);

figure
percent_reduction = (x{2}-x{1})./x{1};
plot(FZ,percent_reduction*100);
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Percent Reduction in Peak Lateral Force','FontSize',15);
title('Peak Lateral Force Decrease (P = 10, IA = 0)',...
    'FontSize',18);

%% Cornering Stiffness Comparison

% change F_y = K_yalpha*-0.0174533 in lateralforce_pure

for i = 1:2
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    Fyplot = lateralforce_pure(Xbestcell,13,FZ,10,0);
    x{i} = Fyplot;
    plot(FZ,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Cornering Stiffness (lb/deg)','FontSize',15);
title('Cornering Stiffness Comparison (P = 10, IA = 0)',...
    'FontSize',18);

figure
percent_reduction = (x{2}-x{1})./x{1};
plot(FZ,percent_reduction*100);
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Percent Reduction in Cornering Stiffness','FontSize',15);
title('Cornering Stiffness Reduction (P = 10, IA = 0)',...
    'FontSize',18);

%% Peak Fy Tire Pressure Sensitivity

figure

for i = 1:2
    Xbestcell = parameters{i};
    P = linspace(10,14,1000).';
    %peakFy = -lateralforce_pure(Xbestcell,12,150,10,0);
    Fyplot = -lateralforce_pure(Xbestcell,12,250,P,0);%/peakFy;
    plot(P,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Tire Pressure (psi)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Peak Lateral Force (FZ = 250, IA = 0)',...
    'FontSize',18);

%% Cornering Stiffness Tire Pressure Sensitivity

figure

% change F_y = K_yalpha*-0.0174533 in lateralforce_pure

for i = 1:2
    Xbestcell = parameters{i};
    P = linspace(10,14,1000).';
    %peakFy = -lateralforce_pure(Xbestcell,12,150,10,0);
    Fyplot = lateralforce_pure(Xbestcell,12,250,P,0);%/peakFy;
    plot(P,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Tire Pressure (psi)','FontSize',15);
ylabel('Cornering Stiffness (lb/deg)','FontSize',15);
title('Cornering Stiffness (FZ = 250, IA = 0)',...
    'FontSize',18);

%% Tire Pressure Plots

figure

for j = [10 12 14]
    Xbestcell = parameters{6};
    alpha = linspace(-15,15,1000).';
    Fyplot = lateralforce_pure(Xbestcell,alpha,150,j,0);
    plot(alpha,Fyplot,'Linewidth',3,'DisplayName',['P = ' num2str(j)]);
    hold on
end

legend
xlabel('Slip Angle (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Lateral Force (FZ = 150, IA = 0)',...
    'FontSize',18);

% %plotting non-tested parameters
% alpha3 = linspace(-15,15,1000).';
% P_input2 = [10];
% IA_input2 = [0];
% FZ_input2 = [150];
% [alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2);
% scatter(alpha2,Fy2,'DisplayName','Raw Data');

%% Parameter loading
% 1: Hoosier 18.0x7.5-10 R25B, 8 in rim
% 2: Hoosier 16.0x7.5-10 R25B, 8 in rim

load('Fy_pure_parameters_run24_camber2.mat')
parameters{1} = Xbestcell;
load('Fy_pure_parameters_run6.mat')
parameters{2} = Xbestcell;

%% Camber Plots

figure

for j = [0 2 4]
    Xbestcell = parameters{4};
    alpha = linspace(-15,15,1000).';
    Fyplot = lateralforce_pure(Xbestcell,alpha,150,12,j);
    plot(alpha,Fyplot,'Linewidth',3,'DisplayName',['Camber = ' num2str(j)]);
    hold on
end

legend
xlabel('Slip Angle (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Lateral Force (FZ = 150, P = 12, IA = 0)',...
    'FontSize',18);

%% Peak Fy Tire Camber Sensitivity

figure

for i = 1:2
    Xbestcell = parameters{i};
    IA = linspace(0,2,1000).';
    peakFy = -lateralforce_pure(Xbestcell,12,150,12,0);
    Fyplot = -lateralforce_pure(Xbestcell,12,150,12,IA)/peakFy;
    plot(IA,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Camber (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Peak Lateral Force (FZ = 150, P = 12)',...
    'FontSize',18);
%% Parameter loading
% 1: Hoosier 18.0x6.0-10 R25B, 6 in rim
% 2: Hoosier 18.0x6.0-10 R25B, 7 in rim
% 3: Hoosier 6.0/18.0-10 LC0, 6 in rim
% 4: Hoosier 6.0/18.0-10 LC0, 7 in rim
% 5: Hoosier 18.0x7.5-10 R25B, 7 in rim
% 6: Hoosier 18.0x7.5-10 R25B, 8 in rim
% 7: Avon 7.0/16.0-10, 7 in rim
% 8: Avon 7.0/16.0-10, 8 in rim

load('Fx_pure_parameters_raw36_final.mat')
parameters{1} = Xbestcell;
load('Fx_pure_parameters_raw39_final.mat')
parameters{2} = Xbestcell;
load('Fx_pure_parameters_raw29_final.mat')
parameters{3} = Xbestcell;
load('Fx_pure_parameters_raw33_final.mat')
parameters{4} = Xbestcell;
load('Fx_pure_parameters_run35_final.mat')
parameters{5} = Xbestcell;
load('Fx_pure_parameters_run38_final.mat')
parameters{6} = Xbestcell;
load('Fx_pure_parameters_run27_final.mat')
parameters{7} = Xbestcell;
load('Fx_pure_parameters_run30_final.mat')
parameters{8} = Xbestcell;

%% Plot settings
linS = {'-','-','--','--',':',':','-.','-.'};
label = {'Hoosier 18.0x6.0-10 R25B, 6 in rim',...
    'Hoosier 18.0x6.0-10 R25B, 7 in rim',...
    'Hoosier 6.0/18.0-10 LC0, 6 in rim',...
    'Hoosier 6.0/18.0-10 LC0, 7 in rim',...
    'Hoosier 18.0x7.5-10 R25B, 7 in rim',...
    'Hoosier 18.0x7.5-10 R25B, 8 in rim',...
    'Avon 7.0/16.0-10, 7 in rim',...
	'Avon 7.0/16.0-10, 8 in rim'};

%% Fy plotter

figure
for i = 1:8
    Xbestcell = parameters{i};
    kappa = linspace(-0.15,0.15,1000).';
    Fxplot = longitudinalforce_pure(Xbestcell,kappa,150,12,0);
    plot(kappa,Fxplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend
xlabel('Slip Ratio','FontSize',15);
ylabel('Longitudinal Force (lb)','FontSize',15);
title('Longitudinal Force Comparison (FZ = 150, P = 12, SA = 0)',...
    'FontSize',18);

%% Peak Fx

figure
for i = 1:8
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    Fxplot = longitudinalforce_pure(Xbestcell,0.14,FZ,12,0);
    plot(FZ,Fxplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Longitudinal Force (lb)','FontSize',15);
title('Peak Longitudinal Force Comparison (P = 12, IA = 0)',...
    'FontSize',18);

%% Peak Fx Tire Pressure Sensitivity

figure

for i = 6
    Xbestcell = parameters{i};
    P = linspace(10,14,1000).';
    %peakFy = -lateralforce_pure(Xbestcell,12,150,10,0);
    Fxplot = longitudinalforce_pure(Xbestcell,0.14,150,P,0);
    plot(P,Fxplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Tire Pressure (psi)','FontSize',15);
ylabel('Longitudinal Force (lb)','FontSize',15);
title('Peak Longitudinal Force (FZ = 150, IA = 0)',...
    'FontSize',18);

%% Tire Pressure Plots

figure

for j = [10 11 12]
    Xbestcell = parameters{i};
    kappa = linspace(-0.15,0.15,1000).';
    Fxplot = longitudinalforce_pure(Xbestcell,kappa,150,j,0);
    plot(kappa,Fxplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',['P = ' num2str(j)]);
    hold on
end

legend
xlabel('Slip Ratio','FontSize',15);
ylabel('Longitudinal Force (lb)','FontSize',15);
title('Longitudinal Force (FZ = 150, IA = 0)',...
    'FontSize',18);

