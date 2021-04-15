%% Finding optimal torque transfer thingy

clear
setup_paths
carCell = carConfig(); % generate all cars to sim over
numCars = size(carCell,1);

car = carCell{1,1};
%%--------%% Input Radius %%--------%%
radius = 8.33;

% initial guesses
steer_angle_guess = 3; % degrees
throttle_guess = .2; %try 1 
long_vel_guess = 5; 
lat_vel_guess = 0.1;
lat_accel_value = 17;
yaw_rate_guess = long_vel_guess/radius;

kappa_1_guess = 0;
kappa_2_guess = 0;
kappa_3_guess = 0.001;
kappa_4_guess = 0.001;
T_3_guess = 16.5;
T_4_guess = 16.5;

x0 = [steer_angle_guess,throttle_guess,long_vel_guess,lat_vel_guess,yaw_rate_guess,kappa_1_guess,...
    kappa_2_guess,kappa_3_guess,kappa_4_guess, T_3_guess, T_4_guess];

% bounds
steer_angle_bounds = [0,25];
throttle_bounds = [0,1]; 
long_vel_bounds = [0,55];
lat_vel_bounds = [-3,3];
yaw_rate_bounds = [0,2];
kappa_1_bounds = [0,0];
kappa_2_bounds = [0,0];
kappa_3_bounds = [0,0.5];
kappa_4_bounds = [0,0.5];
T_3_bounds = [0,33];
T_4_bounds = [0,33];

A = [];
b = [];
Aeq = [0 0 0 0 0 1 0 0 0 0 0
       0 0 0 0 0 0 1 0 0 0 0
       0 0 0 0 0 0 0 0 0 1 1];
beq = [0 0 33];
lb = [steer_angle_bounds(1),throttle_bounds(1),long_vel_bounds(1),lat_vel_bounds(1),...
    yaw_rate_bounds(1),kappa_1_bounds(1),kappa_2_bounds(1),kappa_3_bounds(1),kappa_4_bounds(1),T_3_bounds(1),T_4_bounds(1)];
ub = [steer_angle_bounds(2),throttle_bounds(2),long_vel_bounds(2),lat_vel_bounds(2),...
    yaw_rate_bounds(2),kappa_1_bounds(2),kappa_2_bounds(2),kappa_3_bounds(2),kappa_4_bounds(2),T_3_bounds(2),T_4_bounds(2)];

% scaling
scaling_factor = ones(1,11);
x0 = x0./scaling_factor;
lb = lb./scaling_factor;
ub = ub./scaling_factor;

% maximize long_ccel
f = @(P) -long_accel_calc(car, P);                                 

% longitudinal acceleration constrained to equal zero
% velocity divided by yaw rate constrained to equal inputted radius
% used for solving skidpad (optimizing velocity for zero longitudinal acceleration
constraint = @(P) car.constraint9(P, lat_accel_value, 5); 

% default algorithm is interior-point
options = optimoptions('fmincon','MaxFunctionEvaluations',5000,'ConstraintTolerance',1e-2,...
    'StepTolerance',1e-10,'Display','notify-detailed');

[x,fval,exitflag] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,constraint,options);

[engine_rpm,beta,lat_accel,long_accel,yaw_accel,wheel_accel,omega,current_gear,...
Fzvirtual,Fz,alpha,T,Fy] = car.equations(x);

            
obj = car;
P = x;
steer_angle = P(1);
throttle = P(2); % -1 for full braking, 1 for full throttle
long_vel = P(3); % m/s
lat_vel = P(4); % m/sb
yaw_rate = P(5); % equal to long_vel/radius (v/r)            
kappa = P(6:9);
% note: 1 = front left tire, 2 = front right tire
%       3 = rear left tire, 4 = rear right tire
%T = P(10:11);

beta = atan(lat_vel/long_vel)*180/pi;
long_vel
steer_angle
lat_accel
long_accel

function [long_accel] = long_accel_calc(obj, P)
    [~,~,~,long_accel,~,~,~,~,~,~,~,~,~] = obj.equations(P);
end

%% Corner Radius Plotter

%load("track_autocross_2019.mat");
%corner_curvatures = curvature_apexes(arclength,curvature);
%corner_radii = abs(corner_curvatures).^-1;
%corner_radii_clipped = corner_radii(corner_radii<200);%% remove non-corners with massive radii

%plot frequency of each corner radius
%histogram(corner_radii_clipped,20);
%xlabel("Corner Radius")
%ylabel("Frequency")
%title("2019 Autocross Track Lincoln")


 %8.33m, 11.5m, 17m, 22.5m



