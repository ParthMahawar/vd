%% Car Parameters
clear; close all; clc

load('dampingcurves.mat');
% input car parameters
car = testCar();
car.k = 200*4.45*39.37; % N/m
car.k_tf = 657*4.45*39.37; % tire stiffness (N/m)
car.k_tr = car.k_tf;
car.Ixx = 60;
car.Iyy = 82;
car.c_compression = [0 0.5 7 10; 0 20 40 50]'; % damping curves ([in/s lbf])
car.c_rebound = [0 0.5 7 10; 0 20 40 50]';
car.k_rf = 3000*6.474; % Nm/rad
car.k_rr = 0*6.474; % Nm/rad
car.MR_F = MR18Front;
car.MR_R = MR18Front;

%% Modal Analysis
tirePos = zeros(1,4);
tireVel = zeros(1,4);
[m,c,k] = calc_MCK(car,tirePos,tireVel);
[V,D] = eig(inv(m)*k);

% sort eigenvalues/vectors
[d,ind] = sort(diag(D));
Ds = D(ind,ind);
Vs = V(:,ind);

% natural frequencies
omega = diag(sqrt(Ds))/(2*pi);

for n = 1:7
    subplot(7,1,n)
    [maximum,index] = max(abs(Vs(:,n)));
    plot(Vs(:,n)/maximum*sign(Vs(index,n)));
    legend(['\omega =' num2str(omega(n))])
    set(gcf,'Position',[184 66 600 714]);
    set(gca,'XTick',1:7,'XTickLabel',{'Bounce','Roll','Pitch',...
        'FL Tire' 'FR Tire' 'RL Tire' 'RR Tire'})
end
