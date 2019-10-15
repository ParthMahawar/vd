function car = testCar()

% car parameters (updated 5/25/19)
carParams = struct();
carParams.mass = 179.2; % not including driver (395 lb)
carParams.driver_weight = 68; % (150 lb)
carParams.accel_driver_weight = 68; % (150 lb)
carParams.wheelbase = 1.5494; % 61 in
carParams.weight_dist = 0.54; % percentage of weight in rear
carParams.track_width = 1.1938; % (47 in)
carParams.wheel_radius = 0.173; %0.1956; % loaded radius (7.7 in)
carParams.cg_height = 0.3048; % (12 in)
carParams.roll_center_height_front = 0.0254; % (1 in)
carParams.roll_center_height_rear = 0.0889; % (3.5 in)
carParams.R_sf = 0.4; % proportion of roll stiffness in front (not same as LLTD)
carParams.I_zz = 83.28; %kg-m^2

% aero parameters (updated 5/25/19)
aeroParams = struct();
aeroParams.cda = 1.73; % m^2
aeroParams.cla = 3.77; % m^2
aeroParams.distribution = 0.5; % proportion of downforce in front

% KTM engine parameters (updated 5/1/19)
eParams = struct();
eParams.redline = 11500; 
eParams.shift_point = 8500; % approximate
% these parameters are non-iterable
eParams.gears = [32/16 30/18 28/20 26/22 24/24]; % updated KTM450
eParams.primary_reduction = 76/32; % KTM450
eParams.torque_fn = KTM450();
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
load('Fy_combined_parameters_16x10x7.5_LC0.mat'); % F_y combined magic formula parameters
tireParams.Fy_parameters = cell2mat(Xbestcell);
tireParams.friction_scaling_factor = 0.53; % scales tire forces to account for test/road surface difference

aP = aeroParams;
cP = carParams;
bP = Bparams;
DTP = DTparams;
eP = eParams;
tP = tireParams;
aero = Aero(aP.cda,aP.cla,aP.distribution);
powertrain = Powertrain(eP.redline,eP.shift_point,eP.gears,eP.primary_reduction,eP.torque_fn,eP.shift_time,DTP.final_drive,cP.wheel_radius,...
    DTP.drivetrain_efficiency,DTP.G_d1,DTP.G_d2_overrun,DTP.G_d2_driving, ...
    bP.brake_distribution, bP.max_braking_torque);
tire = Tire2(tP.gamma, tP.p_i,tP.Fx_parameters,tP.Fy_parameters,tP.friction_scaling_factor);
car = Car(cP.mass+cP.driver_weight,cP.wheelbase,cP.weight_dist,cP.track_width,cP.wheel_radius,...
        cP.cg_height,cP.roll_center_height_front,cP.roll_center_height_rear,cP.R_sf,cP.I_zz,aero,...
        powertrain,tire); 