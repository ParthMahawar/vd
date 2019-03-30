%% Load Data

clear all;clc

% import mat file from motec
load('ConstantRadius1_processed.mat');

% input variable names of channels 
data = {engine_rpm,lat_accel,long_accel,shockpot_FR,shockpot_RR,shockpot_RL,...
    shockpot_FL,wheelspeed_FL,wheelspeed_FR,yaw_rate,steer_angle,time'};

load('ConstantRadius2_processed.mat');
data2 = {engine_rpm,lat_accel,long_accel,shockpot_FR,shockpot_RR,shockpot_RL,...
    shockpot_FL,wheelspeed_FL,wheelspeed_FR,yaw_rate,steer_angle,time'};
for i = 1:numel(data)
    data{i} = [data{i}; data2{i}];
end

load('ConstantRadius3_processed.mat');
data2 = {engine_rpm,lat_accel,long_accel,shockpot_FR,shockpot_RR,shockpot_RL,...
    shockpot_FL,wheelspeed_FL,wheelspeed_FR,yaw_rate,steer_angle,time'};
for i = 1:numel(data)
    data{i} = [data{i}; data2{i}];
end

load('ConstantRadius4_processed.mat');
data2 = {engine_rpm,lat_accel,long_accel,shockpot_FR,shockpot_RR,shockpot_RL,...
    shockpot_FL,wheelspeed_FL,wheelspeed_FR,yaw_rate,steer_angle,time'};
for i = 1:numel(data)
    data{i} = [data{i}; data2{i}];
end

remove_indices = abs(wheelspeed_FL)<5;

for i = 1:numel(data)
    data{i}(remove_indices) = [];
end

% rename variables
engine_rpm = data{1};
lat_accel = data{2};
long_accel = data{3};
shockpot_FR = data{4};
shockpot_RR = data{5};
shockpot_RL = data{6};
shockpot_FL = data{7};
wheelspeed_FL = data{8};
wheelspeed_FR = data{9};
yaw_rate = data{10};
steer_angle = data{11};
time = data{12};

%% Plotting
figure
scatter(lat_accel,steer_angle)
xlim([-inf 0]);
ylim([-inf 0]);
xlabel('Lateral Acceleration (g)')
ylabel('Steer Angle (deg)')
title('From accelerometer')

velocity = (wheelspeed_FL+wheelspeed_FR)/2*0.44704; % m/s
pseudo_lat_accel = yaw_rate.*velocity*0.44704*pi/180/9.81; %-velocity.^2/30/9.81;
figure
scatter(pseudo_lat_accel,steer_angle)
xlim([-1.5 0]);
ylim([-inf 0]);
xlabel('Lateral Acceleration (g)')
ylabel('Steer Angle (deg)')
title('From yaw rate * velocity')

pseudo_lat_accel = -velocity.^2/30/9.81;
figure
scatter(pseudo_lat_accel,steer_angle)
xlim([-inf 0]);
ylim([-inf 0]);
xlabel('Lateral Acceleration (g)')
ylabel('Steer Angle (deg)')
title('From v^2/r')
