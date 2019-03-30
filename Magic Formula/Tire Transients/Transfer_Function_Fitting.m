%% Measured Data
%close all
%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,100,150,200,250

P_input = [12];           
IA_input = [0];           
FZ_input = [150]; 
V_input = [2];
SA_input = [1];

[alpha, Fy, Fz, Mz, ~, gamma, pi, testrange, time, velocity] = TireParser_Transients(P_input, IA_input, FZ_input, V_input, SA_input);

Mz_ss = mean(Mz(round(numel(Mz)/2):end));
Fy_ss = mean(Fy(round(numel(Fy)/2):end));

%% Fy Transfer Function Fitting 
data = iddata(Fy/Fy_ss,alpha,.01);
D1 = tfest(data,3,0);

opt = stepDataOptions;
opt.StepAmplitude = mean(alpha);
[y1,t1] = step(D1,linspace(0,time(end),1000),opt);

figure
plot(t1,y1,'LineWidth',5,'Color','k')
hold on
scatter(time, alpha/mean(alpha),70,'.')
hold on
scatter(time, Fy/Fy_ss,70,'.')

title('18.0 x 7.5 R25B  Time vs Lateral Force','FontSize',18);
xlabel('Time (s)','FontSize',15);
ylabel('Normalized Lateral Force','FontSize',15);
legend({'Transfer Function','Slip Angle Step Input','Lateral Force Raw Data'}...
    ,'FontSize', 15, 'Location','southeast')

[y,t] = step(D1,linspace(0,time(end),numel(time)));

s = find(y >= .66*y(end));
timeconstant = t(s(1));
%% Mz Transfer Function Fitting

figure
data = iddata(Mz/Mz_ss,alpha,.01); 
D2 = tfest(data,3,3); 

% uncomment for matching characteristic equation to Fy transfer function
% [num,den] = tfdata(D1);
% init_sys = idtf(NaN(1,4),den{1});
% init_sys.Structure.Denominator.Free = [0 0 0 0];
% D2 = tfest(data,init_sys);

opt = stepDataOptions;
opt.StepAmplitude = mean(alpha);
[y2,t2] = step(D2,linspace(0,time(end),1000),opt);

plot(t2,y2)
hold on
plot(time, alpha/mean(alpha))
hold on
plot(time, Mz/Mz_ss)
title('18.0 x 7.5 R25B Time vs Aligning Torque');
xlabel('Time (s)');
ylabel('Normalized Aligning Torque');
legend('Transfer Function','Slip Angle Step Input','Aligning Torque Raw Data','Location','best')

figure
plot(t1,y1)
hold on
plot(t2,y2)
title('18.0 x 7.5 R25B Lateral Force vs Aligning Torque');
xlabel('Time (s)');
ylabel('Normalized Values');
legend('Normalized Lateral Force','Normalized Aligning Torque','Location','best')
axis([0 2.5 0 2.5]);

