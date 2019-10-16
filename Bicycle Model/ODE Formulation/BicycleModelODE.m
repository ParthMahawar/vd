%% Inputs
weight_dist = 0.3; % rearwards
m = 242.6718; % mass, kg
L = 1.524; % wheelbase

u = 20; % forward velocity, m/s

I_zz = 83.28; % yaw inertia, kg-m^2
C_f = 136*4.448*180/pi; % front cornering stiffness, N/deg @ 150 lb
C_r = 136*4.448*180/pi; % rear cornering stiffness, N/deg @ 150 lb
g = 9.806; % gravitational constant, m/sec^2

N = 16; % steering ratio, made up for now lol

a = L*weight_dist; % front axle to cg, m
b = L*(1-weight_dist); % rear axle to cg, m

D_f = (g*m*b)/(L*C_f); % front cornering compliance, deg/g
D_r = (g*m*a)/(L*C_r); % rear cornering compliance, deg/g
%%

load('Fy_pure_parameters_run24_17.mat')
Xcell = Xbestcell;

BicycleODE2 = @(t,x) BicycleODE(t,x,a,b,u,delta,C_f,C_r,m,I_zz,Xcell);

[t,x] = ode45(BicycleODE2,linspace(0,0.4,1000),[0;0]);

r = x(:,1);
beta = x(:,2);

figure
plot(r)

figure
plot(beta)
