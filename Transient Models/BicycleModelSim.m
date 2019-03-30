%% Inputs
m = 1500; % mass, kg
L = 2500; % wheelbase
a = 1.1; % front axle to cg, m
b = 1.6; % rear axle to cg, m

u = 140/3.6; % forward velocity, m/s

I_zz = 2500; % yaw inertia, kg-m^2
C_f = 55000; % front cornering stiffness, N/rad
C_r = 60000; % rear cornering stiffness, N/rad

%% Plotting

sim BicycleModelSimulink
close_system

figure
plot(beta.time,beta.signals.values)
figure
plot(r.time,r.signals.values)
figure
plot(ay.time,ay.signals.values)

%% Frequency Response Estimation

% Specify portion of model to estimate:
beta_io(1)=linio('BicycleModelSimulink/Steer Angle',1,'input');
beta_io(2)=linio('BicycleModelSimulink/Car',2,'output');

ay_io(1)=linio('BicycleModelSimulink/Steer Angle',1,'input');
ay_io(2)=linio('BicycleModelSimulink/Car',3,'output');

r_io(1)=linio('BicycleModelSimulink/Steer Angle',1,'input');
r_io(2)=linio('BicycleModelSimulink/Car',4,'output');

% Specify operating point for linearization and estimation:
car_spec = operspec('BicycleModelSimulink');
op = findop('BicycleModelSimulink',car_spec);

% Linearize the model:
sys = linearize('BicycleModelSimulink',op,io);

% Estimate the frequency response of the car model
input = frest.Sinestream('Frequency',logspace(-1,2,50));
[beta_sysest] = frestimate('BicycleModelSimulink',op,beta_io,input);
[ay_sysest] = frestimate('BicycleModelSimulink',op,ay_io,input);
[r_sysest] = frestimate('BicycleModelSimulink',op,r_io,input);

% Bode plot
bodeopt = bodeoptions;
bodeopt.MagScale = 'linear';
bodeopt.MagUnits = 'abs';
bodeopt.FreqUnits = 'Hz';

figure
bode(ay_sysest,r_sysest,beta_sysest,bodeopt)
legend('Lateral Acceleration','Yaw Velocity','Sideslip Angle')

%% Transfer Function Estimation

data = iddata(beta.signals.values,delta.signals.values)
sys = tfest(data,2)

