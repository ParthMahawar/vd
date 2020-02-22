function [lat_accel,K,steer_angle,beta,alpha_f,alpha_r,Fz_f,Fz_r,yaw_rate,long_vel,Fy_f,Fy_r] = ...
    UndersteerGradient_constantVel(car,velocity)

counter = 1;

[x_table, x_guess] = constant_radius(30,velocity,car);
x0 = x_guess;

[x_ss,lat_accel,lat_accel_guess] = max_lat_accel_ss(velocity,0,car);

radii = linspace(40,(1/(lat_accel*9.81))*velocity^2+0.5,30);

steer_angle = zeros(size(radii));
beta = zeros(size(radii));
lat_accel = zeros(size(radii));
long_vel = zeros(size(radii));
yaw_rate = zeros(size(radii));

for i = radii
    x0(3) = velocity;
    x0(5) = velocity/i;
    
    [x_table, x_guess] = constant_radius(i,velocity,car,x0);    
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
    Fy_f(counter) = (x_table{1,'Fy_1'}+x_table{1,'Fy_2'})/2;
    Fy_r(counter) = (x_table{1,'Fy_3'}+x_table{1,'Fy_4'})/2;
    counter = counter+1;
end

K = diff(steer_angle-180/pi*car.W_b*yaw_rate./long_vel)./diff(lat_accel/9.81);

end
