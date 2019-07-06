function [time_vec,ending_vel,long_accel_vector,long_vel_vector] = straight(long_vel,length,...
    long_vel_interp,long_accel_interp,max_vel,accelCar)
% inputs: length of straight and starting velocity
% outputs: time and ending velocity 

distance = [0 linspace(0,length,10000)];
long_accel_vector = zeros(1,10000);
long_vel_vector = zeros(1,10000);

obj = accelCar.powertrain;
current_gear = 1;
shift_time_cumulative = 0;
start_shifting = false;

time = zeros(1,10000);
for i = 1:10000 
    
    if long_vel>obj.switch_gear_velocities(current_gear)
        start_shifting = true;
    end
    
    if start_shifting ...
            && shift_time_cumulative < obj.shift_time
        long_accel = -accelCar.aero.drag(long_vel)/accelCar.M;
        shift_time_cumulative = shift_time_cumulative+time(i-1);
    else
        % basic kinematics equations
        long_accel = lininterp1(long_vel_interp,long_accel_interp,long_vel);
    end
    
    if long_vel == max_vel
        long_accel = 0;
    end
     
    % reset shift time counter
    if shift_time_cumulative>obj.shift_time
        start_shifting = false;
        shift_time_cumulative = 0;
        current_gear = current_gear+1;
    end
    
    long_accel_vector(i) = long_accel;
    long_vel_initial = long_vel;
    long_vel_vector(i) = long_vel_initial;
    long_vel = sqrt(long_vel^2+2*long_accel*(distance(i+1)-distance(i)));
    % limit top speed to max velocity
    long_vel = min(long_vel,max_vel);
    time(i) = 2*(distance(i+1)-distance(i))/(long_vel+long_vel_initial);
    
end

time_vec = cumsum(time);

% velocity at end of straight
ending_vel = long_vel;

end

