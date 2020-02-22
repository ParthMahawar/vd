%% Parameter loading
% 1: Hoosier 18.0x7.5-10 R25B, 8 in rim
% 2: Hoosier 16.0x7.5-10 R25B, 8 in rim
% 3: Hoosier 16.0x7.5-10 LC0B, 8 in rim

setup_paths

load('Lapsim_Fy_pure_parameters_1654run24_camber2.mat')
%load('Fy_pure_parameters_run16.mat')
parameters{1} = Xbestcell;
load('Fy_pure_parameters_1965run6.mat')
parameters{2} = Xbestcell;
load('Fy_pure_parameters_run15.mat')
parameters{3} = Xbestcell;

%% Plot settings
linS = {'-','-','-','--',':',':','-.','-.'};
label = {'Hoosier 18.0x7.5-10 R25B, 8 in rim',...
    'Hoosier 16.0x7.5-10 R25B, 8 in rim'...
    'Hoosier 16.0x7.5-10 LC0, 8 in rim'};

%% Fy plotter

figure
for i = 1:3
    for j = 250
    Xbestcell = parameters{i};
    alpha = linspace(-12,12,1000).';
    Fyplot = lateralforce_pure(Xbestcell,alpha,j,12,0);
    plot(alpha,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
    end
end

legend
xlabel('Slip Angle (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Lateral Force Comparison (FZ = 250, P = 12, IA = 0)',...
    'FontSize',18);

%% 
P_input2 = [12];
IA_input2 = [0];
FZ_input2 = [50 150 250];
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2,'A1654run24.mat');
scatter(alpha2,Fy2,'DisplayName','18 RB25 raw data');
title('18x7.5-10 R25B')

%% 
P_input2 = [12];
IA_input2 = [0];
FZ_input2 = [50 150 250];
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2, 'A1965run6.mat');
scatter(alpha2,Fy2,'DisplayName','16 RB25 raw data');
title('16x7.5-10 R25B')

%% 
P_input2 = [12];
IA_input2 = [0];
FZ_input2 = [50 150 250];
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2, 'A1965run15.mat');
scatter(alpha2,Fy2,'DisplayName','16 LC0 raw data');
title('16x7.5-10 LC0')


%% Peak Fy 


figure
for i = 1:3
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    Fyplot = -lateralforce_pure(Xbestcell,12,FZ,12,0);
    x{i} = Fyplot;
    plot(FZ,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Peak Lateral Force Comparison (P = 12, IA = 0)',...
    'FontSize',18);


figure
percent_reduction = (x{3}-x{1})./x{1};
plot(FZ,percent_reduction*100,'DisplayName','Loss from 18" R25B to 16" LC0');
hold on
percent_reduction = (x{2}-x{1})./x{1};
plot(FZ,percent_reduction*100,'DisplayName','Loss from 18" R25B to 16" R25B');
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Percent Reduction in Peak Lateral Force','FontSize',15);
title('Peak Lateral Force Decrease (P = 12, IA = 0)',...
    'FontSize',18);
legend('Location','best')

legend('18 R25B to 16 R25B','18 R25B to 16 LC0')

%% Cornering Stiffness Comparison

for i = 1:3
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    [~, Ky] = lateralforce_pure(Xbestcell,13,FZ,12,0);
    x{i} = Ky;
    plot(FZ,Ky,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Cornering Stiffness (lb/deg)','FontSize',15);
title('Cornering Stiffness Comparison (P = 12, IA = 0)',...
    'FontSize',18);

figure
percent_reduction = (x{2}-x{1})./x{1};
plot(FZ,percent_reduction*100);
hold on

percent_reduction = (x{3}-x{1})./x{1};
plot(FZ,percent_reduction*100);

xlabel('Normal Load (lb)','FontSize',12);
ylabel('Percent Reduction in Cornering Stiffness','FontSize',12);
title('Cornering Stiffness Reduction (P = 12, IA = 0)',...
    'FontSize',15);

legend('18 R25B to 16 R25B','18 R25B to 16 LC0')

%% Peak Fy Tire Pressure Sensitivity

figure

for i = 1:3
    Xbestcell = parameters{i};
    P = linspace(10,14,1000).';
    Fyplot = -lateralforce_pure(Xbestcell,12,250,P,0);
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

for i = 1:3
    Xbestcell = parameters{i};
    P = linspace(10,14,1000).';
    [~,Ky] = lateralforce_pure(Xbestcell,12,250,P,0);
    plot(P,Ky,'Linewidth',3,'LineStyle',linS{i},...
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
    Xbestcell = parameters{3};
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

% %% Parameter loading
% % 1: Hoosier 18.0x7.5-10 R25B, 8 in rim
% % 2: Hoosier 16.0x7.5-10 R25B, 8 in rim
% 
% load('Fy_pure_parameters_run24_camber2.mat')
% parameters{1} = Xbestcell;
% load('Fy_pure_parameters_run6.mat')
% parameters{2} = Xbestcell;

%% Camber Plots

figure

for j = [0 2 4]
    Xbestcell = parameters{2};
    alpha = linspace(-12,12,1000).';
    Fyplot = lateralforce_pure(Xbestcell,alpha,250,12,j);
    plot(alpha,Fyplot,'Linewidth',3,'DisplayName',['Camber = ' num2str(j)]);
    hold on
end

legend
xlabel('Slip Angle (deg)','FontSize',15);
ylabel('Lateral Force (lb)','FontSize',15);
title('Lateral Force (FZ = 250, P = 12, IA = 0)',...
    'FontSize',18);

%% Peak Fy Tire Camber Sensitivity

figure

for i = 1:3
    Xbestcell = parameters{i};
    IA = linspace(-2,2,1000).';
    peakFy = -lateralforce_pure(Xbestcell,-12,250,12,0);
    Fyplot = -lateralforce_pure(Xbestcell,-12,250,12,IA)/peakFy;
    plot(IA,Fyplot,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Camber (deg)','FontSize',15);
ylabel('Normalized Lateral Force','FontSize',15);
title('Peak Lateral Force (FZ = 250, P = 12)',...
    'FontSize',18);

%% Camber Stiffness

figure

for i = 1:3
    Xbestcell = parameters{i};
    FZ = linspace(50,250,1000).';
    [~,~,K_ygamma] = lateralforce_pure(Xbestcell,13,FZ,12,0);
    plot(FZ,-K_ygamma,'Linewidth',3,'LineStyle',linS{i},...
        'DisplayName',label{i});
    hold on
end

legend('Location','best')
xlabel('Normal Load (lb)','FontSize',15);
ylabel('Camber Stiffness (lb/deg)','FontSize',15);
title('Camber Stiffness vs Normal Load',...
    'FontSize',18);


