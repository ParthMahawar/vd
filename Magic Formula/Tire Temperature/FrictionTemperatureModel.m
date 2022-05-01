%% Temperature and Slip Angle vs Time
clear; close all; clc

load('A1654raw24.mat')
figure

plot(ET,TSTC)
hold on

load('A1965raw6.mat')
plot(ET,TSTC)

load('A1965raw15.mat')
plot(ET,TSTC)

xlim([0 500])
xlabel('Time (s)')
ylabel('Tire Center Surface Temperature (F)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

%%
figure
ET_input = [0 10000];
SA_input = [-11];
P_input = 12;
IA_input = 0;
FZ_input = 250;

file_name = 'A1654raw24.mat';
%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_1 = TSTC_out;
mu_1 = abs(FY_out./FZ_out);

indices = (TSTC_out<130 | TSTC_out > 137);
TSTC_1 = TSTC_1(indices);
mu_1 = mu_1(indices);

TSTC_1 = [TSTC_1; 90*ones(100,1); 80*ones(100,1)];
mu_1 = [mu_1; 1.82*ones(100,1); 1.78*ones(100,1)];

scatter(TSTC_1,mu_1);
hold on

file_name = 'A1965raw6.mat';
%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_2 = TSTC_out;
mu_2 = abs(FY_out./FZ_out);
scatter(TSTC_2,mu_2);

file_name = 'A1965raw15.mat';
ET_input = [0 90];
SA_input = [-12 12];

%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_3 = TSTC_out;
mu_3 = abs(FY_out./FZ_out);
scatter(TSTC_3,mu_3);
xlabel('Temperature (F)')
ylabel('Friction Coefficient')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

%% Curve Fitting
figure
ft = fittype( 'rat22' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.Normalize = 'on';
opts.StartPoint = [0.347712671277525 0.149997253831683 0.586092067231462 0.262145317727807 0.0444540922782385];
f1 = fit(TSTC_1, mu_1, ft, opts );
plot(f1,TSTC_1,mu_1)
hold on

figure
opts.StartPoint = [1, 0, 1, 0, 0.5];
f2 = fit(TSTC_2,mu_2, ft, opts );
plot(f2,TSTC_2,mu_2)

figure
f3 = fit(TSTC_3,mu_3, ft, opts );
plot(f3,TSTC_3,mu_3)

figure
plot1 = plot(f1,'g',TSTC_1,mu_1,'g.');
hold on
plot2 = plot(f2,'k',TSTC_2,mu_2,'k.');
plot3 = plot(f3,'b',TSTC_3,mu_3,'b.');

%operating temperature
xline(130,'-',{'Operating Temperature'},...
    'LabelVerticalAlignment', 'bottom', 'fontSize', 12);
xline(140);

legend([plot1(2) plot2(2) plot3(2)],{'18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0'},'Location','southeast')
title('Tire Friction Coefficient vs Temperature');
xlabel('Temperature (F)')
ylabel('Friction Coefficient')

figure
plot(f1)
hold on
plot(f2,'-b')
hold on
plot(f3,'--g')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

%% Normalization
figure

x = linspace(80,160,1000);
X=(x-114.9)/18.48;
mu1_norm = (2.218.*X.^2+3.203.*X+4.972)./(X.^2+0.9495.*X+2.102);
plot(x,mu1_norm)
hold on
X=(x-146.2)/13.08;
mu2_norm = (2.014.*X.^2+8.438.*X+20.65)./(X.^2+3.54.*X+8.566);
plot(x,mu2_norm)
X=(x-147.1)/12.13;
mu3_norm = (1.976.*X.^2+6.725.*X+13.25)./(X.^2+2.896.*X+5.196);
plot(x,mu3_norm)
xlabel('Temperature (F)')
ylabel('Friction Coefficient')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

figure
X=(x-114.9)/18.48;
mu1_norm = 0.3897*(2.218.*X.^2+3.203.*X+4.972)./(X.^2+0.9495.*X+2.102);
X=(x-146.2)/13.08;
mu2_norm = 0.4147*(2.014.*X.^2+8.438.*X+20.65)./(X.^2+3.54.*X+8.566);
X=(x-147.1)/12.13;
mu3_norm = 0.3846*(1.976.*X.^2+6.725.*X+13.25)./(X.^2+2.896.*X+5.196);
plot(x,mu1_norm)
hold on
plot(x,mu2_norm)
plot(x,mu3_norm)
xlabel('Temperature (F)')
ylabel('Normalized Friction Coefficient')

%% Finding Operating temperature of tests
figure
ET_input = [240 10000];
SA_input = [-12];
P_input = [12];
IA_input = 0;
FZ_input = [50 150 250];

file_name = 'A1654raw24.mat';
%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_1 = TSTC_out;
mu_1 = abs(FY_out./FZ_out);

scatter(TSTC_1,mu_1);
hold on

file_name = 'A1965raw6.mat';
%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_2 = TSTC_out;
mu_2 = abs(FY_out./FZ_out);
scatter(TSTC_2,mu_2);

file_name = 'A1965raw15.mat';
SA_input = [-12];

%[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser3(ET_input,P_input, IA_input, FZ_input, SA_input, file_name);

TSTC_3 = TSTC_out;
mu_3 = abs(FY_out./FZ_out);
scatter(TSTC_3,mu_3);
xlabel('Temperature (F)')
ylabel('Friction Coefficient')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

mean(TSTC_1)
mean(TSTC_2)
mean(TSTC_3)

%% Final Functions

x = mean(TSTC_1);
X=(x-114.9)/18.48;
mu1_ref = 0.3897*(2.218.*X.^2+3.203.*X+4.972)./(X.^2+0.9495.*X+2.102);
x = mean(TSTC_2);
X=(x-146.2)/13.08;
mu2_ref = 0.4147*(2.014.*X.^2+8.438.*X+20.65)./(X.^2+3.54.*X+8.566);
x = mean(TSTC_3);
X=(x-147.1)/12.13;
mu3_ref = 0.3846*(1.976.*X.^2+6.725.*X+13.25)./(X.^2+2.896.*X+5.196);

%% Final
x = linspace(80,160,1000);
X=(x-114.9)/18.48;
mu1_norm = 0.3897/mu1_ref*(2.218.*X.^2+3.203.*X+4.972)./(X.^2+0.9495.*X+2.102);
X=(x-146.2)/13.08;
mu2_norm = 0.4147/mu2_ref*(2.014.*X.^2+8.438.*X+20.65)./(X.^2+3.54.*X+8.566);
X=(x-147.1)/12.13;
mu3_norm = 0.3846/mu3_ref*(1.976.*X.^2+6.725.*X+13.25)./(X.^2+2.896.*X+5.196);

figure
plot(x,mu1_norm)
hold on
plot(x,mu2_norm)
plot(x,mu3_norm)
xlabel('Temperature (F)')
ylabel('Friction Coefficient Scaling')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0','Location','southeast')
