clear; clc
% run this from the same directory as setup_paths
try 
    setup_paths
catch
    error('run this from same directory as setup_paths. Should be one directory up');
end

% input car parameters
car = testCar();
car.k = 200*4.45*39.37; % N/m
car.c = 800; % N*s/m
car.Ixx = 60;
car.Iyy = 82;
car.k_rf = 0; % Nm/rad
car.k_rr = 0; % Nm/rad
car.TSmpc = .003; %has to be multiple of TSdyn
car.TSdyn = .0005;
car.Jm = 0; car.Jw = 1;
n = 8000; % number of timesteps

% steering/throttle input
steerDeg = 3;
steer = deg2rad(steerDeg)*[zeros(1,n/8) ones(1,7*n/8)];

%time = 0:car.TSmpc:car.TSmpc*(n-1);
%steer = steer.*sin((2*pi)*time);
%steer(1:3000) = 0;

%steer = chirp(time,0,time(end),2,'linear',-90);
%steer =  deg2rad(steerDeg)*[zeros(1,3000) steer(1:end-3000)];

throttle = zeros(1,n);
%throttle = 0.1*ones(1,n);
%throttle = [0*ones(1,n/2) 1*ones(1,n/4) -1*ones(1,n/4)];
% throttle = [zeros(1,n/4) ones(1,2*n/4) -ones(1,n/4)];
uArr = [steer; throttle];

% initial vehicle states (vector of 14 values)
% 1: yaw angle 2: yaw rate 3: long velocity 4: lat velocity
% 5: x position of cg 6: y position of cg
% 7:  FL angular position 8:  FL angular velocity
% 9:  FR angular position 10: FR angular velocity
% 11: RL angular position 12: RL angular velocity
% 13: RR angular position 14: RR angular velocity

x0 = zeros(14,1);
v0 = 26; % initial velocity (m/s)
x0(3) = v0; 
x0([8 10 12 14]) = v0/car.R; % wheel velocities

% main dynamics solver 
% outputs data: struct containing time histories of states, 
% roll/pitch angles, tire normal loads, and wheel displacements
data = fullDynamics(car,uArr,x0,n);

%% Plotting

xArr = data.xArr; xdotArr = data.xdotArr; FzArr = data.FzArr; yArr = data.yArr; 

% 1: yaw angle 2: yaw rate 3: long velocity 4: lat velocity
% 5: x position of cg 6: y position of cg
% 7:  FL angular position 8:  FL angular velocity
% 9:  FR angular position 10: FR angular velocity
% 11: RL angular position 12: RL angular velocity
% 13: RR angular position 14: RR angular velocity

time = 0:car.TSmpc:car.TSmpc*(n-1);
yaw_angle = xArr(1,:);
yaw_rate = xArr(2,:);
long_vel = xArr(3,:);
lat_vel = xArr(4,:);
lat_accel = long_vel.*yaw_rate+xdotArr(4,:);
beta = rad2deg(atan(lat_vel./long_vel));

bounce = yArr(1,:);
phi = yArr(2,:);
theta = yArr(3,:);

ic = 1000;
figure(1);clf
plot(xArr(5,:),xArr(6,:));
hold on
plot(xArr(5,ic),xArr(6,ic),'o');
title('XY Pos');grid
xlabel('X Position');
ylabel('Y Position');

figure(2);clf
plot(time,sqrt(long_vel.^2 + lat_vel.^2));
title('speed');grid

figure(3);clf
plot(time,rad2deg(phi)); hold on
plot(time,rad2deg(theta));grid
title('phi and theta, deg');
legend('phi','theta','Location','best');

figure(4);clf
plot(time,rad2deg(steer)) %deg
title('steering angle');grid

figure(5);clf
for i =1:4
    plot(time,FzArr(i,:));hold on
end
grid
title('Fz'); legend('1','2','3','4');

figure(6);clf
plot(time,bounce);
title('Roll Center Height');

disp('done');
fprintf("phi: %0.2f\n",rad2deg(phi(end-10)));
fprintf("theta: %0.2f\n",rad2deg(theta(end-10)));

figure(7); clf
plot(time,long_vel.*yaw_rate)
hold on
plot(time,lat_accel)
title('Lateral Acceleration')

figure(8); clf
plot(time,yaw_rate)
title('Yaw Rate')

figure(9); clf
plot(time,steer/max(abs(steer)))
hold on
plot(time,-phi/max(phi))
plot(time,lat_accel/max(lat_accel))
plot(time,yaw_rate/max(yaw_rate))
plot(time,-beta/max(beta))
legend('Steer','Roll','Lat Accel','Yaw Rate','Sideslip Angle')

figure(10); clf
plot(time,beta)
title('Beta')

%% Transfer Function Estimation

data = iddata(beta',steer',car.TSmpc);
beta_sys = tfest(data,2,1);

data = iddata(lat_accel',steer',car.TSmpc);
ay_sys = tfest(data,2,2);

data = iddata(yaw_rate',steer',car.TSmpc);
r_sys = tfest(data,2,1);

% Bode plot
bodeopt = bodeoptions;
bodeopt.MagScale = 'linear';
bodeopt.MagUnits = 'abs';
bodeopt.FreqScale = 'linear';
bodeopt.FreqUnits = 'Hz';
bodeopt.PhaseMatching = 'on';
bodeopt.PhaseMatchingFreq = 0;
bodeopt.PhaseMatchingValue = 0;

% figure
% bode(ay_sys/dcgain(ay_sys),r_sys/dcgain(r_sys),beta_sys/dcgain(beta_sys),bodeopt)
% xlim([0 4]);
% legend('Lateral Acceleration','Yaw Velocity','Sideslip Angle')
% 
% figure
% bode(ay_sys,r_sys,beta_sys,bodeopt)
% xlim([0 4]);
% legend('Lateral Acceleration','Yaw Velocity','Sideslip Angle')
%%
load('FreqResponseData3.mat')
sstxy_1 = sstxy;
sstxy_2 = sstxy2;
load('FreqResponseData4.mat')
sstxy_1 = sstxy_1+sstxy;
sstxy_2 = sstxy_2+sstxy2;
load('FreqResponseData5.mat')
a = 80;
sstxy_1(1:a) = sstxy_1(1:a)+sstxy(1:a);
sstxy_2(1:a) = sstxy_2(1:a)+sstxy(1:a);

sstxy = sstxy_1/3;
sstxy2 = sstxy_2/3;

figure
subplot(2,1,1)
plot(time,steer/max(abs(steer)),'Color',[0.9290 0.6940 0.1250])
hold on
plot(time,lat_accel/max(lat_accel),'Color',[0.8500 0.3250 0.0980])
plot(time,yaw_rate/max(yaw_rate),'Color',[0 0.4470 0.7410])

legend('Steer Angle','Lateral Acceleration','Yaw Rate','Location','Northwest')
xlabel('Time (s)')
ylabel('Normalized Amplitude')
%title('Comparison of Simulated and Measured Vehicle Frequency Response')

subplot(2,1,2)

[mag,phase,w] = bode(ay_sys,linspace(0,2*(2*pi),1000),bodeopt);
plot(squeeze(w)/(2*pi), squeeze(phase)-360, 'Color',[0.8500 0.3250 0.0980]);
[mag2,phase2,w2] = bode(r_sys,linspace(0,2*(2*pi),1000),bodeopt);
hold on
plot(squeeze(w)/(2*pi), squeeze(phase2), 'Color',[0 0.4470 0.7410]);

hold on

hold on
plot(f,1.3*180/pi*angle(sstxy/sstxy(1)),'--','Color',[0.8500 0.3250 0.0980])
hold on
plot(f,180/pi*angle(sstxy2/sstxy2(1)),'--','Color',[0 0.4470 0.7410])
xlim([0 1.75]);

xlabel('Frequency (Hz)')
ylabel('Phase (deg)')

legend('Lateral Acceleration (Simulated)','Yaw Rate (Simulated)','Lateral Acceleration (Measured)','Yaw Rate (Measured)','Location','Southwest')

