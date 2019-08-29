%% Parameters
clear
setup_paths
carCell = carConfig(); % generate all cars to sim over
numCars = size(carCell,1);

car = carCell{1,1};

%% Parameter Sweep
radius = 10; % m 

for i = 1:numCars
    car = carCell{i,1};
    [lat_accel,K,steer_angle,beta,alpha_f,alpha_r,Fz_f,Fz_r] = UndersteerGradient(car,radius);
    lat_accel_vec{i} = lat_accel;
    K_vec{i} = K;
    steer_angle_vec{i} = steer_angle;
    beta_vec{i} = beta;
    alpha_f_vec{i} = alpha_f;
    alpha_r_vec{i} = alpha_r;
    
    [x_table_skid, max_vel_skid, skidpad_time, skid_guess] = max_skidpad_vel(radius,car); % constrained to 0 long_accel
    max_lat_accel_trim1{i} = x_table_skid{1,'lat_accel'}/9.81;
    [x_ss,lat_accel,long_accel,~] = max_lat_accel(radius,car); % irregardless of long accel
    max_lat_accel_trim2{i} = lat_accel/9.81;
    [x_table_ss,lat_accel,yaw_accel,lat_accel_guess] = max_lat_accel_yaw(radius,car);
    max_lat_accel_untrimmed{i} = lat_accel/9.81;
    Cn_residual{i} = yaw_accel/(car.M*9.81*car.W_b);
end

alpha = [x_table_skid{1,'alpha_1'};x_table_skid{1,'alpha_2'};x_table_skid{1,'alpha_3'};x_table_skid{1,'alpha_4'}];
kappa = [x_table_skid{1,'kappa_1'};x_table_skid{1,'kappa_2'};x_table_skid{1,'kappa_3'};x_table_skid{1,'kappa_4'}];
Fz = [x_table_skid{1,'Fz_1'};x_table_skid{1,'Fz_2'};x_table_skid{1,'Fz_3'};x_table_skid{1,'Fz_4'}];
[Fx,Fy,~] = car.tireForce(x_table_skid{1,'steer_angle'}*pi/180,alpha,kappa,Fz);

%% 

for alpha_vec = -1:-1:-9
Fz_vec = linspace(0,1200,1000);
for i = 1:1000
    Fy_vec(i) = car.tire.F_y(alpha_vec,0,Fz_vec(i));
    hold on
end
    plot(Fz_vec,Fy_vec)
end

%% Radius Sweep
% radius_vec= 10:5:30; % m 
% 
% for i = 1:numel(radius_vec)
%     radius = radius_vec(i);
%     car = carCell{1,1};
%     [lat_accel,K,steer_angle,beta,alpha_f,alpha_r] = UndersteerGradient(car,radius);
%     lat_accel_vec{i} = lat_accel;
%     K_vec{i} = K;
%     steer_angle_vec{i} = steer_angle;
%     beta_vec{i} = beta;
%     alpha_f_vec{i} = alpha_f;
%     alpha_r_vec{i} = alpha_r;
%     
%     [x_ss,lat_accel,long_accel,~] = max_lat_accel(radius,car);
%     max_lat_accel_trim{i} = lat_accel/9.81;
%     [x_table_ss,lat_accel,yaw_accel,lat_accel_guess] = max_lat_accel_yaw(radius,car);
%     max_lat_accel_untrimmed{i} = lat_accel/9.81;
%     Cn_residual{i} = yaw_accel/(car.M*9.81*car.W_b);
% end

%% Plotting
figure
for i = 1:numCars
    plot(lat_accel_vec{i}/9.81,steer_angle_vec{i})
    hold on
    xlabel('Lateral Accel (g)')
    ylabel('Steer Angle (deg)')
end
%legend('0.51','0.53','0.55','0.57')
legend('LLTD = 0.3', '0.4', '0.5', '0.6')

figure
for i = 1:numCars
    plot(lat_accel_vec{i}(2:end)/9.81, K_vec{i});
    hold on
    xlabel('Lateral Accel (g)','FontSize',15)
    ylabel('Understeer Gradient (deg/g)','FontSize',15)
end
%legend('0.51','0.53','0.55','0.57')
legend('LLTD = 0.3', '0.4', '0.5', '0.6')

%%
figure
for i = 1:numCars
    plot(lat_accel_vec{i}(2:end)/9.81, K_vec{i});
    hold on
    plot(tony1{i}(2:end)/9.81, tony2{i});
    xlabel('Lateral Accel (g)','FontSize',15)
    ylabel('Understeer Gradient (deg/g)','FontSize',15)
end
%legend('0.51','0.53','0.55','0.57')
legend('Track Width = 48 & 46','Track Width = 47 & 47')



