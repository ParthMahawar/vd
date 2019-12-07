close all;clear;clc;
setup_paths
%load('A1965raw15.mat')
load('A1965raw6.mat')
%load('A1654raw24.mat')

%% Thermodynamic Model
range = 1:40000;

V = 25; % mph
V = V*2.44704; % m/s

Vx = 0;
Vy = V*tand(SA);

P = 10; % psi

% T: vector of temperatures (C)
% [road, tread, carcass, inflation gas, ambient]
road = 80; % C (RST)
tread = 85; % C (TSTC)
carcass = 85; % C
gas = 85; % C
ambient = 80.5; % C

T0 = [road tread carcass gas ambient]';
T = T0;

params = [1.5];

% tspan = ET(range);
% [t,soltrue] = ode45(@(t,T) thermoModelODE(T,Vx,Vy,FX*4.45,FY*4.45,FZ*4.45,V,P,params,ET,t),tspan,T0);
% 
% plot(t,soltrue(:,2))
% hold on
% plot(tspan,TSTC(range))
%% Fitting

tspan = ET(range);
y0 = T0;

yvalstrue = TSTC(range)';

tspan = tspan(1:500:end);
yvalstrue = yvalstrue(1:500:end);

r = optimvar('r',1,"LowerBound",0.5,"UpperBound",2);
myfcn = fcn2optimexpr(@RtoODE,r,tspan,y0);
obj = sum(sum((myfcn(2,:) - yvalstrue).^2));

prob = optimproblem("Objective",obj);
r0.r = 0.2;
[rsol,sumsq] = solve(prob,r0);
disp(rsol.r)

%% Plotting
figure
plot(ET(range),T_matrix(:,1))
hold on
plot(ET(range),T_matrix(:,2))
plot(ET(range),T_matrix(:,3))
plot(ET(range),T_matrix(:,4))
plot(ET(range),T_matrix(:,5))

xlabel('Time (s)')
ylabel('Temperature (C)')
legend('Road','Tread','Carcass','Gas','Ambient')

figure
plot(ET(range),T_matrix(:,2))
hold on
plot(ET(range),TSTC(range))

figure
plot(ET(range),dTdt_matrix(:,1))
hold on
plot(ET(range),dTdt_matrix(:,2))
plot(ET(range),dTdt_matrix(:,3))
plot(ET(range),dTdt_matrix(:,4))
plot(ET(range),dTdt_matrix(:,5))

xlabel('Time (s)')
ylabel('dT/dt (C/s)')
legend('Road','Tread','Carcass','Gas','Ambient')


