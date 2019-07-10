function carCell = carConfig()

% car parameters (updated 5/25/19)
carParams = struct();
carParams.mass = 179.2; % not including driver (395 lb)
carParams.driver_weight = 68; % (150 lb)
carParams.accel_driver_weight = 68; % (150 lb)
carParams.wheelbase = 1.524; % 60 in
carParams.weight_dist = 0.51; % percentage of weight in rear
carParams.track_width = 1.27; % (50 in)
carParams.wheel_radius = 0.1956; % loaded radius (7.7 in)
carParams.cg_height = 0.3048; % (12 in)
carParams.roll_center_height_front = 0.0254; % (1 in)
carParams.roll_center_height_rear = 0.0889; % (3.5 in)
carParams.R_sf = 0.31; % proportion of roll stiffness in front (not same as LLTD)
carParams.I_zz = 83.28; %kg-m^2

% aero parameters (updated 5/25/19)
aeroParams = struct();
aeroParams.cda = 1.73; % m^2
aeroParams.cla = 3.77; % m^2
aeroParams.distribution = 0.45; % proportion of downforce in front

% engine parameters (updated 5/1/19)
eParams = struct();
eParams.redline = 12500; 
eParams.shift_point = 10500; % approximate
% these parameters are non-iterable
eParams.gears = [2.0, 1.63, 1.33, 1.14, 0.95]; % KTM350
% eParams.gears = [32/17, 30/19, 28/21, 26/23]; % KTM450
% eParams.gears = [2.583, 2, 1.57, 1.444, 1.286]; % Honda CBR600RR 2007-09
% eParams.gears = [2.75, 2, 1.667, 1.444, 1.304]; % Yamaha R6 2006-11
% eParams.gears = [2.786, 2.053, 1.714, 1.5, 1.348]; % Suzuki GSX-R600 2006-10
eParams.primary_reduction = 3.04; % KTM350
% eParams.primary_reduction = 74/29; % KTM450
% eParams.primary_reduction = 2.111; % CBR600RR
% eParams.primary_reduction = 2.073; % R6
% eParams.primary_reduction = 1.974; % R600

% !!! FIRST DATAPOINT MUST BE (0,0) !!!
eParams.torque_fn = KTM350(); % contains torque curve
% eParams.torque_fn = KTM450();
% eParams.torque_fn = CBR600RR();
% eParams.torque_fn = R6();
% eParams.torque_fn = R600();
eParams.shift_time = 0.050; % seconds FOR UPSHIFT ONLY; 150ms for downshift

% drivetrain parameters (updated 5/1/19)
DTparams = struct();
DTparams.final_drive = 40/11; % drivetrain sprocket ratio
DTparams.drivetrain_efficiency = 0.92; % scales torque value
DTparams.G_d1 = 0; % differential torque transfer offset due to internal friction
DTparams.G_d2_overrun = 0; % differential torque transfer gain in overrun (not used right now)
DTparams.G_d2_driving = 0; % differential torque transfer gain on power

% brake parameters (updated 5/1/19)
Bparams = struct();
Bparams.brake_distribution = 0.7; % proportion of brake torque applied to front
Bparams.max_braking_torque = 800; % total braking torque

% tire parameters (updated 5/1/19)
tireParams = struct();
tireParams.gamma = 0; % camber angle
tireParams.p_i = 12; % pressure
% these parameters are non-iterable
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
tireParams.Fx_parameters = cell2mat(Xbestcell);
load('Fy_combined_parameters_run6_new.mat'); % F_y combined magic formula parameters
tireParams.Fy_parameters = cell2mat(Xbestcell);

% cell array of gridded parameters
[carCell] = parameters_loop(carParams,aeroParams,eParams,DTparams,Bparams,tireParams);