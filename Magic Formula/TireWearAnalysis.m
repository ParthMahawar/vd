close all;clc;
setup_paths
%% Temperature and Slip Angle vs Time
load('A1965raw15.mat')
figure
ax1 = subplot(3,1,1);
plot(ET,TSTC)
hold on

load('A1965raw16.mat')
plot(ET,TSTC)

xlim([0 300])
xlabel('Time (s)')
ylabel('Tire Center Surface Temperature (F)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

ax2 = subplot(3,1,2);
plot(ET,SA)

xlim([0 300])
xlabel('Time (s)')
ylabel('Slip Angle (deg)')

ax3 = subplot(3,1,3);

load('A1965raw15.mat')
plot(ET,FY)

load('A1965raw16.mat')
plot(ET,FY)
linkaxes([ax1,ax2,ax3],'x')

%%
figure
load('A1654raw24.mat')
figure
plot(SA(1:10000),FY(1:10000))
hold on

load('A1965raw6.mat')
plot(SA(1:10000),FY(1:10000))

%xlim([0 300])
xlabel('Time (s)')
ylabel('Tire Center Surface Temperature (F)')
legend('18x7.5-10 R25B','16x7.5-10 R25B')

%% Good Stuff
 
figure
load('A1654raw24.mat')

plot(ET,FY)
hold on
load('A1965raw6.mat')

plot(ET,FY)
xlim([0 300])

hold on
load('A1965raw15.mat')
plot(ET,FY)

figure
plot(ET,TSTC)
xlim([0 300])

figure
plot(TSTC,FY)

%% Good Stuff 2
figure
ET_input = [0 90];
SA_input = -11;

file_name = 'A1654raw24.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(ET_out,FY_out)
hold on

file_name = 'A1965raw6.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(ET_out,FY_out)

file_name = 'A1965raw15.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(ET_out,FY_out)
xlabel('Time (s)')
ylabel('Lateral Force (lb)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

%% Good Stuff 3
figure
ET_input = [400 900];
SA_input = -11;

file_name = 'A1654raw24.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(TSTC_out,smooth(FY_out,10))
hold on

file_name = 'A1965raw6.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(TSTC_out,smooth(FY_out,10))

file_name = 'A1965raw15.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(TSTC_out,smooth(FY_out,1))
xlabel('Temperature (F)')
ylabel('Lateral Force (Lb)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

%% Good Stuff 4

figure
ET_input = [77 80];
SA_input = 0;

file_name = 'A1654raw24.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(SA_out,FY_out)
hold on

ET_input = [40 43];
SA_input = 0;
file_name = 'A1965raw6.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(SA_out,FY_out)

ET_input = [30 33];
SA_input = 0;

file_name = 'A1965raw15.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
scatter(SA_out,FY_out)
xlabel('Slip Angle (deg)')
ylabel('Lateral Force (Lb)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')

figure
ET_input = [74 77];
SA_input = 0;

file_name = 'A1654raw24.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
plot(TSTC_out)
hold on
save('TemperatureCompensated_18R25B.mat','SA_out', 'FY_out', 'FZ_out', 'P_out', 'TSTC_out', 'ET_out',  'IA_out','index')

ET_input = [40 43];
SA_input = 0;
file_name = 'A1965raw6.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out,  IA_out,index] = TireTemperatureParser2(ET_input, SA_input, file_name);
plot(TSTC_out)
save('TemperatureCompensated_16R25B.mat','SA_out', 'FY_out', 'FZ_out', 'P_out', 'TSTC_out', 'ET_out',  'IA_out','index')

ET_input = [30 33];
SA_input = 0;

file_name = 'A1965raw15.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, IA_out, index] = TireTemperatureParser2(ET_input, SA_input, file_name);
plot(TSTC_out)
xlabel('Index')
ylabel('Temperature (F)')
legend('18x7.5-10 R25B','16x7.5-10 R25B','16x7.5-10 LC0')
save('TemperatureCompensated_16LC0.mat','SA_out', 'FY_out', 'FZ_out', 'P_out', 'TSTC_out', 'ET_out', 'IA_out', 'index')


%%
hold on
load('A1965raw6.mat')

plot(ET,FY)
xlim([0 90])

hold on
load('A1965raw15.mat')
plot(ET,FY)

figure
plot(ET,TSTC)
xlim([0 90])

figure
plot(TSTC,FY)

%%
load('A1654run24.mat')
figure
subplot(2,1,1)
plot(ET,TSTC)
hold on

load('A1965run6.mat')
plot(ET,TSTC)

%xlim([0 300])
xlabel('Time (s)')
ylabel('Tire Center Surface Temperature (F)')
legend('18x7.5-10 R25B','16x7.5-10 R25B')

subplot(2,1,2)
%plot(ET,SA)

xlim([0 300])
xlabel('Time (s)')
ylabel('Slip Angle (deg)')

%% Good Stuff 4
P_input = 12;
IA_input = 0;
FZ_input = [250];
SA = 10;
file_name = 'A1654run24.mat';

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA, file_name);

plot(SA_out,TSTC_out)
hold on
file_name = 'A1965run15.mat';

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA, file_name);
plot(SA_out,TSTC_out)
xlabel('Slip Angle (deg)')
ylabel('Tire Center Surface Temp (F)')
legend('18x7.5-10 R25B','16x7.5-10 LC0')
title('FZ = 250, P = 12',...
    'FontSize',15);



%%
file_name = 'A1654run24.mat';
[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA, file_name);

plot(SA_out,FY_out)
hold on
file_name = 'A1965run6.mat';

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA, file_name);
plot(SA_out,FY_out)

%%
figure
subplot(2,1,1)
plot(ET_out,TSTC_out)
subplot(2,1,2)
plot(ET_out,SA_out)

%%
P_input = 10;
IA_input = 0;
FZ_input = 150;
SA = 12;

[SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA, file_name);
plot(SA_out,TSTC_out)

figure
plot(TSTC_out,-FY_out)

