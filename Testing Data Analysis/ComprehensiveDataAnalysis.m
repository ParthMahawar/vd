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

time = [0,inf]; %time range to be plotted

%% Car Data

C = carConfig();
car = C{1,1};

car.MR_F = 0.74;
car.MR_R = 1;

%% Filter Data

% moving mean filter on selected data values
variablesToFilter = {'SuspPosFL','SuspPosFR','SuspPosRL','SuspPosRR',...
    'SteeredAngle', 'WheelSpdRL','WheelSpdRR','WheelSpdFL','WheelSpdFR'};
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
title('Wheel position vs time'), grid

%% heave, roll, pitch, front heave, rear heave

T.Heave = (T.SuspPosFL + T.SuspPosFR + T.SuspPosRL + T.SuspPosRR)/4;
T.Front_Heave = (T.SuspPosFL + T.SuspPosFR)/2;
T.Rear_Heave = (T.SuspPosRL + T.SuspPosRR)/2;
T.Pitch = asin((T.Front_Heave-T.Rear_Heave)/car.W_b);
T.Roll = (T.SuspPosFL + T.SuspPosFR + T.SuspPosRL + T.SuspPosRR)/(2*car.t_f);

%% time selection
figure
subplot(3,1,1)
plotLine(T,time,'Heave')
hold on
yyaxis right
plotLine(T,time,'Pitch')
plotLine(T,time,'Roll')
legend

subplot(3,1,2)
plotLine(T,time,'Front_Heave')
hold on
plotLine(T,time,'Rear_Heave')
legend('Interpreter','latex')

subplot(3,1,3)
plotLine(T,time,'SuspPosFL')
hold on
plotLine(T,time,'SuspPosFR')
plotLine(T,time,'SuspPosRL')
plotLine(T,time,'SuspPosRR')
legend

figure
subplot(3,1,1)
plotLine(T,time,'SteeredAngle')
legend

subplot(3,1,2)
plotLine(T,time,'AccelY')
legend

subplot(3,1,3)
plotLine(T,time,'Roll')
legend

figure
subplot(4,1,1)
plotLine(T,time,'BrakePres_F')
hold on
plotLine(T,time,'BrakePres_R')
T.BrakePres_Tot = T.BrakePres_F + T.BrakePres_R;
plotLine(T,time,'BrakePres_Tot')
legend('Interpreter','latex')

subplot(4,1,2)
%plotLine(T,time,'Throttle')
legend

subplot(4,1,3)
plotLine(T,time,'AccelX')
legend

subplot(4,1,4)
plotLine(T,time,'Pitch')
legend
%% Math channels damper velocity + histogram

damperVelFL = diff(T.SuspPosFL);
damperVelFR = diff(T.SuspPosFR);
damperVelRL = diff(T.SuspPosRL);
damperVelRR = diff(T.SuspPosRR);

figure
plot(T.Time(1:end-1), damperVelFL);
hold on
plot(T.Time(1:end-1), damperVelFR);
plot(T.Time(1:end-1), damperVelRL);
plot(T.Time(1:end-1), damperVelRR);
title('Damper velocity vs time'), grid
hold off

damperVelAvg = mean([damperVelFL,damperVelFR,damperVelRL,damperVelRR]');
figure
histogram(damperVelAvg),title('Damper velocity histogram')
%% Understeer gradient fitting (with filtering) & polyfit  Under construction 
t01 = 15;
select1 = (T.Time>t01 & T.Time<(T.Time(end)-t01));
% Ug.wheelspeed_avg = mean([T.WheelSpdRL,T.WheelSpdRR,...
%     T.WheelSpdFL,T.WheelSpdFR]');
% Ug.wheelspeed_avg = Ug.wheelspeed_avg(select1);
% Ug.wheelspeed_avg(Ug.wheelspeed_avg==NaN) = 0;
% Ug.wheelspeed_avg = Ug.wheelspeed_avg';
% f= fit(Ug.wheelspeed_avg,...
%     T.SteeredAngle(select1),'poly2')
% 
% figure
% plot(f,Ug.wheelspeed_avg,T.SteeredAngle(select1))
% title('Steering agle vs Wheel speed'),...
%     xlabel('Wheel speed'),ylabel('Steering angle'),grid
% 
% Gradeant_average = polyval([f.p1,f.p2,f.p3],T.Time(end)) - polyval([f.p1,f.p2,f.p3],t0)

wheelspeed_avg = (mean([T.WheelSpdRL,T.WheelSpdRR,...
    T.WheelSpdFL,T.WheelSpdFR]'));
wheelspeed_avg = wheelspeed_avg(select1);
wheelspeed_avg = wheelspeed_avg';
f = fit(wheelspeed_avg,T.SteeredAngle(select1),'poly1');

figure
plot(f,wheelspeed_avg,T.SteeredAngle(select1)),...
    title('Steering agle vs Wheel speed'),...
    xlabel('Wheel speed'),ylabel('Steering angle'),grid

Gradeant_average = f.p1;
disp('The average under steer grad = '+ string(Gradeant_average))
    
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
%% GG plot
figure
scatter(T,'AccelY','AccelX'), title('GG plot'), xlabel('Lateral Gs')
ylabel('Longitudinal Gs')

%% FUNctions :D 
% this has to have been greg

function [] = plotLine(T, timeRange, name, varargin)
    y = T.(name);
    y = y(T.Time>timeRange(1)&T.Time<timeRange(2));
    x = T.Time(T.Time>timeRange(1)&T.Time<timeRange(2));

    plot(x,y,...
        'displayName', name, varargin{:});
    

end




