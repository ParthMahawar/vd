close all;clear;clc;
setup_paths
load('A1965raw15.mat')
%load('A1654raw24.mat')

%% Thermodynamic Model
range = 1:40000;

V = 25; % mph
V = V*2.44704; % m/s

Vx = 0;
Vy = V*tand(SA);

% T: vector of temperatures (C)
% [road, tread, carcass, inflation gas, ambient]
road = 80; % C (RST)
tread = 85; % C (TSTC)
carcass = 85; % C
gas = 85; % C
ambient = 80.5; % C

T0 = [road tread carcass gas ambient]';
T = T0;

dt = 0.01; % sampling
for i = range
    road = RST(i);
    ambient = AMBTMP(i);
    T(1) = road;
    T(end) = ambient;
    dTdt = thermoModel(T,Vx,Vy(i),FX(i)*4.45,FY(i)*4.45,FZ(i)*4.45);
    dTdt_matrix(i,:) = dTdt;
    T_matrix(i,:) = T;
    T = T+dTdt*dt;
end

%% Plotting
figure
plot(ET(range),T_matrix(:,1))
hold on
plot(ET(range),T_matrix(:,2))
plot(ET(range),T_matrix(:,3))
plot(ET(range),T_matrix(:,4))
plot(ET(range),T_matrix(:,5))

xlabel('Time (s)')
ylabel('Temperature (C)')
legend('Road','Tread','Carcass','Gas','Ambient')

figure
plot(ET(range),T_matrix(:,2))
hold on
plot(ET(range),TSTC(range))

figure
plot(ET(range),dTdt_matrix(:,1))
hold on
plot(ET(range),dTdt_matrix(:,2))
plot(ET(range),dTdt_matrix(:,3))
plot(ET(range),dTdt_matrix(:,4))
plot(ET(range),dTdt_matrix(:,5))

xlabel('Time (s)')
ylabel('dT/dt (C/s)')
legend('Road','Tread','Carcass','Gas','Ambient')


