function carCell = carConfig()

% car parameters (updated 2/4/21)
carParams = struct();
carParams.mass = 179.2 + 5.9; % not including driver (395 lb)
carParams.driver_weight = 68; % (150 lb)
carParams.accel_driver_weight = 68; % (150 lb)
carParams.wheelbase = 1.5494; % 61 in
carParams.weight_dist = 0.54; % percentage of weight in rear
carParams.track_width = 1.1938; % (47 in)
carParams.wheel_radius = 0.1956; % loaded radius (7.7 in)
carParams.cg_height = 0.3048; % (12 in)
carParams.roll_center_height_front = 0.0254; % (1 in)
carParams.roll_center_height_rear = 0.0889; % (3.5 in)
carParams.R_sf = 0.4; % proportion of roll stiffness in front (not same as LLTD)
carParams.I_zz = 83.28; %kg-m^2

% aero parameters (updated 2/4/21)
aeroParams = struct();
aeroParams.cda = 1.73; % m^2
aeroParams.cla = 3.77 + 0.7265; % m^2
aeroParams.distribution = 0.4; % proportion of downforce in front

% KTM engine parameters (updated 5/1/19)
eParams = struct();
eParams.redline = 11500; 
eParams.shift_point = 8500; % approximate
% these parameters are non-iterable
eParams.gears = [32/16 30/18 28/20 26/22 24/24]; % updated KTM450
eParams.primary_reduction = 76/32; % KTM450
eParams.torque_fn = KTM450_RecaroTurbo();
eParams.shift_time = 0.050; % seconds FOR UPSHIFT ONLY; 150ms for downshift

% drivetrain parameters (updated 5/1/19)
DTparams = struct();
DTparams.final_drive = 40/11; % drivetrain sprocket ratio
DTparams.drivetrain_efficiency = 0.87; % scales torque value
DTparams.G_d1 = 0; % differential torque transfer offset due to internal friction
DTparams.G_d2_overrun = 0; % differential torque transfer gain in overrun (not used right now)
TBR = 1;%1:0.5:4;
DTparams.G_d2_driving = (TBR-1)./(2+2*TBR); % differential torque transfer gain on power

% brake parameters (updated 5/1/19)
Bparams = struct();
Bparams.brake_distribution = 0.7;% proportion of brake torque applied to front
Bparams.max_braking_torque = 800; % total braking torque

% tire parameters (updated 5/1/19)
tireParams = struct();
tireParams.gamma = [-1,0,1]; % camber angle
tireParams.p_i = 12; % pressure
% these parameters are non-iterable
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
tireParams.Fx_parameters = cell2mat(Xbestcell);
load('Lapsim_Fy_combined_parameters_1965run15.mat'); % F_y combined magic formula parameters
tireParams.Fy_parameters = cell2mat(Xbestcell);
tireParams.friction_scaling_factor = 1.05*0.55; % scales tire forces to account for test/road surface difference

% cell array of gridded parameters
[carCell] = parameters_loop(carParams,aeroParams,eParams,DTparams,Bparams,tireParams);