function [lat_accel,K,steer_angle,beta,alpha_f,alpha_r,Fz_f,Fz_r] = UndersteerGradient(car,radius)

counter = 1;

[x_table, x_guess] = constant_radius(radius,4,car);
x0 = x_guess;

long_vel_guess = sqrt(1.5*radius*9.81);
[~, max_vel_skid, ~] = max_skidpad_vel(radius,car,x0);

velocities = 3:0.1:max_vel_skid-0.5;
steer_angle = zeros(size(velocities));
beta = zeros(size(velocities));
lat_accel = zeros(size(velocities));
long_vel = zeros(size(velocities));
yaw_rate = zeros(size(velocities));

for i = velocities
    x0(3) = i;
    x0(5) = i/radius;
    
    [x_table, x_guess] = constant_radius(radius,i,car,x0);    
    x0 = x_guess;
    steer_angle(counter) = x_table{1,'steer_angle'};
    beta(counter) = x_table{1,'beta'};
    lat_accel(counter) = x_table{1,'lat_accel'};
    long_vel(counter) = x_table{1,'long_vel'};
    yaw_rate(counter) = x_table{1,'yaw_rate'};
    alpha_f(counter) = (x_table{1,'alpha_1'}+x_table{1,'alpha_2'})/2;
    alpha_r(counter) = (x_table{1,'alpha_3'}+x_table{1,'alpha_4'})/2;
    Fz_f(counter) = (x_table{1,'Fz_1'}+x_table{1,'Fz_2'})/2;
    Fz_r(counter) = (x_table{1,'Fz_3'}+x_table{1,'Fz_4'})/2;

    counter = counter+1;
end

K = diff(steer_angle)./diff(lat_accel/9.81);

end
