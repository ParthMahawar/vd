clear
close all
clc
set(0,'DefaultTextInterpreter','none')
%% Import Data

filename = 'skidpad2_50hz.csv';

% include units
opt = detectImportOptions(filename);
opt.VariableUnitsLine = 16;
T = readtable(filename, opt);

T = T(2:end,:); % remove first row

%% Inputs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% time selection
timeRange = [0,Inf]; %time range to be plotted

% plot selection
bodyMovementPlot = 0;
lateralPlot = 0;
longitudinalPlot = 0;
understeerPlot = 0;
damperVelocityHistogram = 0;
GGPlot = 0;
aeroPlot = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Car Data

C = carConfig();
car = C{1,1};

car.MR_F = 0.74;
car.MR_R = 1;
car.k = 200*4.45*39.37; % N/m ---add k front and k rear


%% Filter Data

% moving mean filter on selected data values
variablesToFilter = {'SuspPosFL','SuspPosFR','SuspPosRL','SuspPosRR',...
    'SteeredAngle', 'WheelSpdRL','WheelSpdRR','WheelSpdFL','WheelSpdFR'};
meanRangeSeconds = 0.25; % s
meanTimestep = mean(diff(T.Time));
order = meanRangeSeconds/meanTimestep;
T(:,variablesToFilter) = ...
    array2table(movmean(table2array(T(:,variablesToFilter)),order,1));

%% Zero Sensors

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
T = setUnits(T, {'wheelPosFL', 'wheelPosFR', 'wheelPosRL', 'wheelPosRR'},...
    getUnits(T, 'SuspPosRR'));

%% Calculate New Channels

% heave, roll, pitch, front heave, rear heave, using shock pot values
T.SuspHeave = (T.wheelPosFL + T.wheelPosFR + T.wheelPosRL + T.wheelPosRR)/4;
T.FrontHeave = (T.wheelPosFL + T.wheelPosFR)/2;
T.RearHeave = (T.wheelPosRL + T.wheelPosRR)/2;
T.SuspPitch = rad2deg((T.RearHeave-T.FrontHeave)/(car.W_b*1000));
T.SuspRoll = rad2deg((T.wheelPosFL + T.wheelPosRL - T.wheelPosFR - T.wheelPosRR)/(2*car.t_f*1000));
T = setUnits(T, {'SuspHeave', 'FrontHeave', 'RearHeave'},...
    getUnits(T, 'wheelPosFL'));
T = setUnits(T, {'SuspPitch', 'SuspRoll'},'deg');

% damperVelocities
T.damperVelFL = [0; diff(T.wheelPosFL)./diff(T.Time)];
T.damperVelFR = [0; diff(T.wheelPosFR)./diff(T.Time)];
T.damperVelRL = [0; diff(T.wheelPosRL)./diff(T.Time)];
T.damperVelRR = [0; diff(T.wheelPosRR)./diff(T.Time)];
T = setUnits(T, {'damperVelFL', 'damperVelFR', 'damperVelRL', 'damperVelRR'},...
    [getUnits(T, 'wheelPosRR') '/s']);

% speed
T.Speed = (T.WheelSpdRL + T.WheelSpdRL + T.WheelSpdRL + T.WheelSpdRL)/4 * 1000/3600; % m/s
T = setUnits(T, 'Speed', 'm/s');

%% time selection plots
if bodyMovementPlot
    figure
    subplot(3,1,1)
    plotLine(T,timeRange,'SuspHeave')
    hold on
    yyaxis right
    plotLine(T,timeRange,'SuspPitch')
    plotLine(T,timeRange,'SuspRoll')
    legend('Interpreter','none')
    grid

    subplot(3,1,2)
    plotLine(T,timeRange,'FrontHeave')
    hold on
    plotLine(T,timeRange,'RearHeave')
    legend('Interpreter','none')
    grid
    
    subplot(3,1,3)
    plotLine(T,timeRange,'wheelPosFL')
    hold on
    plotLine(T,timeRange,'wheelPosFR')
    plotLine(T,timeRange,'wheelPosRL')
    plotLine(T,timeRange,'wheelPosRR')
    legend('Interpreter','none')
    grid

    sgtitle('Body Movement')
end

if lateralPlot
    figure
    subplot(3,1,1)
    plotLine(T,timeRange,'SteeredAngle')
    legend('Interpreter','none')
    
    subplot(3,1,2)
    plotLine(T,timeRange,'AccelY')
    legend('Interpreter','none')
    
    subplot(3,1,3)
    plotLine(T,timeRange,'SuspRoll')
    legend('Interpreter','none')

    sgtitle('Lateral Plot')
end

if longitudinalPlot
    figure
    subplot(4,1,1)
    plotLine(T,timeRange,'BrakePres_F')
    hold on
    plotLine(T,timeRange,'BrakePres_R')
    legend('Interpreter','none')
    
    subplot(4,1,2)
    plotLine(T,timeRange,'TPS')
    legend('Interpreter','none')
    
    subplot(4,1,3)
    plotLine(T,timeRange,'AccelX')
    legend('Interpreter','none')
    
    subplot(4,1,4)
    plotLine(T,timeRange,'SuspPitch')
    legend('Interpreter','none')

    sgtitle('Longitudinal Plot')
end
%% damper velocity histogram
if damperVelocityHistogram
    figure
    subplot(3,2,1)
    histogram(T.damperVelFL), xlim([-100,100])
    title('damperVelFL'), xlabel('velocity (mm/s)')
    subplot(3,2,2)
    histogram(T.damperVelFR), xlim([-100,100])
    title('damperVelFR'), xlabel('velocity (mm/s)')
    subplot(3,2,3)
    histogram(T.damperVelRL), xlim([-100,100])
    title('damperVelRL'), xlabel('velocity (mm/s)')
    subplot(3,2,4)
    histogram(T.damperVelRR), xlim([-100,100])
    title('damperVelRR'), xlabel('velocity (mm/s)')

    subplot(3,1,3);
    hold on
    plotLine(T, timeRange, 'damperVelFL')
    plotLine(T, timeRange, 'damperVelFR')
    plotLine(T, timeRange, 'damperVelRL')
    plotLine(T, timeRange, 'damperVelRR')
    title('Damper velocity vs time'), ylabel('velocity (mm/s)'), grid
end
%% Understeer gradient fitting (with filtering) & polyfit  Under construction 
% add radius calculation and filtering
t01 = 15;
select1 = (T.Time>t01 & T.Time<(T.Time(end)-t01));

f1 = fit(T.Speed(select1),T.SteeredAngle(select1),'poly1');

if understeerPlot
    figure
    plot(f1, T.Speed(select1), T.SteeredAngle(select1)),...
        title(['Steering Angle vs Wheel Speed | Understeer Gradient = ' num2str(f1.p1)]),...
        xlabel('Wheel speed'), ylabel('Steering angle'), grid
end

%% GG plot
if GGPlot
    figure
    scatter(T,'AccelY','AccelX'), title('GG Plot')
    xlabel('Lateral Gs'), ylabel('Longitudinal Gs')
    grid
    xlim([-2,2]), ylim([-2,2])
    hold on
    xline(0),yline(0)
end

%% Aero, plots downforce vs speed, fits CLA
include = ~or(isnan(T.Speed), isnan(T.SuspHeave));
exclude =  T.Speed(include)<1;

%obj.rho/2*(long_vel^2)*obj.cla;

% calculate downforce
totalHeaveStiffness = 2*(350*4.45*39.37)*car.MR_F^2 + ... 
                      2*(250*4.45*39.37)*car.MR_R^2; % N/m
FzTotal = T.SuspHeave(include)/1000*totalHeaveStiffness; % N

ft = fittype(@(a, x) -(a*car.aero.rho/2)*x.^2); 
f2 = fit(T.Speed(include), FzTotal, ft, 'Exclude', exclude);
totalHeaveStiffness = 2*car.k/car.MR_F^2 + 2*car.k/car.MR_R^2;
CLA = round(f2.a);

if aeroPlot
    figure
    plot(f2,T.Speed(include), FzTotal, exclude)
    title(['Steering Angle vs Wheel Speed | CLA = ' num2str(CLA) 'm^2'])
    xlabel('Speed (m/s)');
    ylabel('Force (N)');
    grid
end

% CLA = 2*F/(rho v^2)
% F = (CLA*rho/2)*v^2


%% FUNctions :D 
% this has to have been greg

function varargout = plotLine(T, timeRange, name, varargin)
    y = T.(name);
    y = y(T.Time>timeRange(1)&T.Time<timeRange(2));
    x = T.Time(T.Time>timeRange(1)&T.Time<timeRange(2));

    [varargout{1:nargout}] = plot(x,y,...
        'displayName', name, varargin{:});
    xlabel('time (s)')
    ylabel(getUnits(T, name));
end

function unit = getUnits(T, name)
    i = strcmp(T.Properties.VariableNames, name);
    unit = T.Properties.VariableUnits{i};
end

function T = setUnits(T, names, unit)
    if iscell(names)
        for j = 1:numel(names)
            i = strcmp(T.Properties.VariableNames, names{j});
            T.Properties.VariableUnits{i} = unit;
        end
    else
        i = strcmp(T.Properties.VariableNames, names);
        T.Properties.VariableUnits{i} = unit;
    end
end

