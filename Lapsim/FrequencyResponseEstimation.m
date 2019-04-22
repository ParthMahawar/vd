
%% Transfer Function Estimation

data1 = iddata(beta',steer',car.TSmpc);
beta_sys = tfest(data1,2,1);

data2 = iddata(lat_accel',steer',car.TSmpc);
ay_sys = tfest(data2,2,2);

data3 = iddata(yaw_rate',steer',car.TSmpc);
r_sys = tfest(data3,2,1);

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
plot(squeeze(w)/(2*pi), squeeze(phase)-phase(1), 'Color',[0.8500 0.3250 0.0980]);
[mag2,phase2,w2] = bode(r_sys,linspace(0,2*(2*pi),1000),bodeopt);
hold on
plot(squeeze(w)/(2*pi), squeeze(phase2)-phase2(1), 'Color',[0 0.4470 0.7410]);

hold on

hold on
plot(f,1.3*180/pi*angle(sstxy/sstxy(1)),'--','Color',[0.8500 0.3250 0.0980])
hold on
plot(f,180/pi*angle(sstxy2/sstxy2(1)),'--','Color',[0 0.4470 0.7410])
xlim([0 1.75]);

xlabel('Frequency (Hz)')
ylabel('Phase (deg)')

legend('Lateral Acceleration (Simulated)','Yaw Rate (Simulated)','Lateral Acceleration (Measured)','Yaw Rate (Measured)','Location','Southwest')

