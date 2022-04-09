clear
close all
clc
%% Import Data

filename = 'skidpad2_50hz.csv';

% include units
opt = detectImportOptions(filename);
opt.VariableUnitsLine = 16;
T = readtable(filename, opt);

T = T(2:end,:); % remove first row

%% Car Data

C = carConfig();
car = C{1,1};

car.MR_F = 0.74;
car.MR_R = 1;

%% Filter Data

% moving mean filter on selected data values
variablesToFilter = {'SuspPosFL','SuspPosFR','SuspPosRL','SuspPosRR', 'SteeredAngle'};
meanRangeSeconds = 0.25; % s
meanTimestep = mean(diff(T.Time));
order = meanRangeSeconds/meanTimestep;
T(:,variablesToFilter) = ...
    array2table(movmean(table2array(T(:,variablesToFilter)),order,1));

%% Filter

% zero out wheel positions to their averages for the first 10 seconds
t0 = 10;
T.SuspPosFL = T.SuspPosFL - mean(T.SuspPosFL(T.Time<t0));
T.SuspPosFR = T.SuspPosFR - mean(T.SuspPosFR(T.Time<t0));
T.SuspPosRL = T.SuspPosRL - mean(T.SuspPosRL(T.Time<t0));
T.SuspPosRR = T.SuspPosRR - mean(T.SuspPosRR(T.Time<t0));

T.wheelPosFL = T.SuspPosFL./car.MR_F;
T.wheelPosFR = T.SuspPosFR./car.MR_F;
T.wheelPosRL = T.SuspPosRL./car.MR_R;
T.wheelPosRR = T.SuspPosRR./car.MR_R;

%% Plot More

figure
plot(T.Time, T.wheelPosFL);
hold on
plot(T.Time, T.wheelPosFR);
plot(T.Time, T.wheelPosRL);
plot(T.Time, T.wheelPosRR);


% start_t = 250;
% end_t = 1350;
% m_k = 40;
% 
% accelY = movmean(T.AccelY,m_k) ;
% accelY = accelY(start_t:end_t);
% 
% str_a = movmean(T.SteeredAngle, m_k);
% str_a = str_a(start_t:end_t);
% 
% t = T.Time;
% t = t(start_t:end_t);
% 
% wheelspeed_avg = movmean(mean([T.WheelSpdRL,T.WheelSpdRR,...
%     T.WheelSpdFL,T.WheelSpdFR]'),m_k);
% wheelspeed_avg = wheelspeed_avg(start_t:end_t);
% 
% roll = movmean(T.Roll(start_t:end_t),m_k);
% 
% 
% 
% 
% % speed = medfilt1(1/2.*t.^2.*skidpad_data.AccelX(250:1300),10);
% figure(1)
% scatter(str_a, accelY), title("Steer angle vs Lateral G's"),...
%     xlabel('Stearing angle'),ylabel('Lateral Acceleration (g)'),grid
% figure(2)
% plot(t, str_a), title('Steer angle vs time'),...
%     xlabel('time (s)'),ylabel('Stearing angle'),grid
% figure(3)
% plot(t,accelY), title('Lateral Acceleration vs time'),...
%     xlabel('time (s)'),ylabel('Lateral Acceleration'),grid
% figure(4)
% plot(t,wheelspeed_avg), title('Wheel speed (4 wheel avg) vs time'),...
%     xlabel('time (s)'),ylabel('Wheel speed'),grid
% 
% wheelspeed_avg = wheelspeed_avg';
% f = fit(wheelspeed_avg,str_a,'poly1')
% 
% figure(5)
% plot(f,wheelspeed_avg,str_a), title('Steering agle vs Wheel speed'),...
%     xlabel('Wheel speed'),ylabel('Steering angle'),grid
% figure(6)
% scatter(str_a,roll), title('Steering agle vs roll'),...
%     xlabel('Steering angle'),ylabel('Roll angle'),grid