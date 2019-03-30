%% Inputs
weight_dist = 0.55; % rearwards
m = 242.6718; % mass, kg
L = 1.524; % wheelbase

a = L*weight_dist; % front axle to cg, m
b = L*(1-weight_dist); % rear axle to cg, m

u = 20; % forward velocity, m/s

I_zz = 83.28; % yaw inertia, kg-m^2
C_f = 136*4.448*180/pi/2; % front cornering stiffness, N/deg @ 150 lb
C_r = 136*4.448*180/pi/2; % rear cornering stiffness, N/deg @ 150 lb

load('Fy_pure_parameters_run24_17.mat')
X = cell2mat(Xbestcell);

step_steer = 7*pi/180; % rad

%% Plotting

sim NonLin_BicycleModelSimulink
close_system

figure
plot(beta.time,beta.signals.values)
title('Sideslip Angle Step Response')
figure
plot(r.time,r.signals.values)
title('Yaw Velocity Step Response')
figure
plot(ay.time,ay.signals.values)
title('Lateral Acceleration Step Response')

%% Frequency Response Estimation

% Specify portion of model to estimate:
beta_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
beta_io(2)=linio('NonLin_BicycleModelSimulink/Car',2,'output');

ay_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
ay_io(2)=linio('NonLin_BicycleModelSimulink/Car',3,'output');

r_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
r_io(2)=linio('NonLin_BicycleModelSimulink/Car',4,'output');

% Specify operating point for linearization and estimation:
car_spec = operspec('NonLin_BicycleModelSimulink');
op = findop('NonLin_BicycleModelSimulink',car_spec);

% Linearize the model:
% sys = linearize('BicycleModelSimulink',op,io);

% Estimate the frequency response of the car model
input = frest.Sinestream('Frequency',logspace(-2,3,50));
[beta_sysest] = frestimate('NonLin_BicycleModelSimulink',op,beta_io,input);
[ay_sysest] = frestimate('NonLin_BicycleModelSimulink',op,ay_io,input);
[r_sysest] = frestimate('NonLin_BicycleModelSimulink',op,r_io,input);

% Bode plot
bodeopt = bodeoptions;
bodeopt.MagScale = 'linear';
bodeopt.MagUnits = 'abs';
bodeopt.FreqUnits = 'Hz';

figure
bode(ay_sysest,r_sysest,beta_sysest,bodeopt)
legend('Lateral Acceleration','Yaw Velocity','Sideslip Angle')

%% Transfer Function Estimation

data = iddata(beta.signals.values,delta.signals.values,0.01);
beta_sys = tfest(data,2,1);

data = iddata(ay.signals.values,delta.signals.values,0.01);
ay_sys = tfest(data,2,2);

data = iddata(r.signals.values,delta.signals.values,0.01);
r_sys = tfest(data,2,1);

% Bode plot
bodeopt = bodeoptions;
bodeopt.MagScale = 'linear';
bodeopt.MagUnits = 'abs';
bodeopt.FreqUnits = 'Hz';

figure
bode(ay_sys,r_sys,beta_sys,bodeopt)
legend('Lateral Acceleration','Yaw Velocity','Sideslip Angle')

%% Linearization

ops_tsnapshot = linspace(0,10,50);
T = linearize('NonLin_BicycleModelSimulink',r_io,ops_tsnapshot);
stepplot(T)

%%
% Specify portion of model to estimate:
beta_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
beta_io(2)=linio('NonLin_BicycleModelSimulink/Car',2,'output');

ay_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
ay_io(2)=linio('NonLin_BicycleModelSimulink/Car',3,'output');

r_io(1)=linio('NonLin_BicycleModelSimulink/Steer Angle',1,'input');
r_io(2)=linio('NonLin_BicycleModelSimulink/Car',4,'output');

% Specify operating point for linearization and estimation:
car_spec = operspec('NonLin_BicycleModelSimulink');
op = findop('NonLin_BicycleModelSimulink',car_spec);

%opspec.States(1).SteadyState = 1;
%opspec.States(1).x = beta.signals.values(end);
%opspec.States(2).SteadyState = 1;
%opspec.States(2).x = r.signals.values(end);

% Linearize the model:
r_linsys = linearize('NonLin_BicycleModelSimulink',op,r_io);

plot(r.time,r.signals.values,'o');
hold on
opt = stepDataOptions;
opt.StepAmplitude = step_steer;
step(r_linsys,[], opt);

