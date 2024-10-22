classdef Events2 < handle
    % Calculates results for different dynamic events
    
    properties
        car
        accelCar
        
        autocross_track
        endurance_track
        
        % info
        skidpad
        accel
        autocross
        endurance
        
        times
        points
        
        % contains info used for interpolation in autocross/accel solvers
        interp_info
    end
    
    methods
        function obj = Events2(car,accelCar)
            obj.car = car;
            obj.accelCar = accelCar;
            
            % maps
            load('track_autocross_2023Fixed.mat');
            obj.autocross_track = [arclength; curvature];
            load('track_endurance_2019.mat');
            obj.endurance_track = [arclength; curvature];
            
            % sweep for max velocity for given radius
            % used for interpolation in Autocross
            [x_table_corner_vel,radius_vector,max_vel_corner_vector] = vel_cornering_sweep(obj.car);
            
            obj.interp_info.x_table_corner_vel = x_table_corner_vel;
            obj.interp_info.radius_vector = radius_vector;
            obj.interp_info.max_vel_corner_vector = max_vel_corner_vector;
            
            % sweep for max pure longitudinal acceleration for given velocity
            % used for interpolation in Accel
            [x_table_accel,long_vel_guess,long_accel_matrix] = long_accel_sweep(obj.accelCar);
            obj.interp_info.x_table_accel = x_table_accel;
            obj.interp_info.long_vel_guess = long_vel_guess;
            obj.interp_info.long_accel_matrix = long_accel_matrix;
                        
        end
        
        function [points] = calcTimes(obj)
            obj.Skidpad();
            obj.Accel();
            obj.Autocross();
            obj.Endurance();
            points = obj.computePoints();
        end
        
        function [x_table_skid,maxVel,time] = Skidpad(obj)
            % modelled as pure steady state (no longitudinal acceleration)
            % inner radius of skidpad is 7.625 m
            % width of skidpad is 3 m
            % old lapsim used 8.55 radius - essentially means ~ 1 ft gap from cones

            radius = 8.5;
            [x_table_skid,maxVel,time] = max_skidpad_vel(radius,obj.car);
            obj.times.skidpad = time;
            obj.skidpad.x_table_skid = x_table_skid;
        end
        
        function [time,ending_vel,long_accel_vector,long_vel_vector] = Accel(obj)
            % car starts 0.3 m behind starting line
            % accel is 75 m long
            
             long_vel = 0;
                      
             long_vel_interp = obj.interp_info.long_vel_guess;
             long_accel_interp = obj.interp_info.long_accel_matrix;
             
              [~,ending_vel,~,~] = straight(long_vel,0.3,long_vel_interp,...
                  long_accel_interp,obj.accelCar.max_vel,obj.accelCar);
              
              % starting velocity for accel is ending velocity of 0.3 straight
              long_vel = ending_vel;
            
            [time_vec,ending_vel,long_accel_vector,long_vel_vector] = straight(long_vel,75,...
                long_vel_interp,long_accel_interp,obj.accelCar.max_vel,obj.accelCar);
            obj.times.accel = time_vec(end);
            obj.accel.time_vec = time_vec;
            obj.accel.long_vel_vector = long_vel_vector;
            obj.accel.long_accel_vector = long_accel_vector;
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Track_Solver(obj,arclength,curvature)
            % finds apexes in curvature profile (apex of corner) and finds
            %   max possible velocity at each apex
            % then max accel and max braking are calculated for each
            %   segment between apexes
            % the minimum of the profiles is used to calculate velocity and
            %   acceleration profiles as well as time
            % for more theoretical detail of method see Siegler p. 20 or Brayshaw p. 52

            % find apexes in curvature profile (apex of corner has smallest turn radius)
            [extrema,extrema_indices] = curvature_apexes(arclength,curvature);

            arclength = [0 arclength];

            % find max possible velocity at each apex
            [apex_velocity] = apex_velocities(obj.interp_info.radius_vector,obj.interp_info.max_vel_corner_vector,extrema);
            
            % F_accel/braking(lat_accel,long_vel) returns the max possible accel/braking
            [F_accel,F_braking] = create_scattered_interpolants2(obj.car.longAccelLookup,...
                obj.car.longDecelLookup);

            % Maximum possible acceleration between apexes
            % calculating velocity and acceleration profiles as well as time
            
            % car starts 6 m behind starting line 
            long_vel_interp = obj.interp_info.long_vel_guess;
            long_accel_interp = obj.interp_info.long_accel_matrix;
            [~,ending_vel,~,~] = straight(0,6,long_vel_interp,long_accel_interp,obj.car.max_vel,obj.car);

            % starting velocity is ending velocity of straight
            long_vel = ending_vel; 

            lat_accel_vector_1 = [];
            long_accel_vector_1 = [];
            long_vel_vector_1 = [];
            time_1 = [];

            lat_accel_vector_2 = [];
            long_accel_vector_2 = [];
            long_vel_vector_2 = [];
            time_2 = [];

            last_index_1 = 1;
            last_index_2 = 1;

            % calculates the maximum possible acceleration and braking between each
            %   apex pair
            for j = 1:numel(extrema_indices) % loop through each segment
                % find max acceleration from initial velocity to next apex
                for i = last_index_1:extrema_indices(j) % segment from previous apex to next        
                    % basic kinematics equations
                    lat_accel = long_vel^2*abs(curvature(i));
                    lat_accel_vector_1(i) = lat_accel*sign(curvature(i));
                    long_accel = F_accel(lat_accel,long_vel);
                    if long_vel == obj.car.max_vel
                        long_accel = 0;
                    end
                    long_accel_vector_1(i) = long_accel;
                    long_vel_initial = long_vel;
                    long_vel_vector_1(i) = long_vel_initial;
                    long_vel = sqrt(long_vel^2+2*long_accel*(arclength(i+1)-arclength(i)));
                    % can't exceed max possible velocity for given radius
                    long_vel = min(long_vel,lininterp1(obj.interp_info.radius_vector,obj.interp_info.max_vel_corner_vector,...
                        abs(1/curvature(i)))); 
                    time_1(i) = 2*(arclength(i+1)-arclength(i))/(long_vel+long_vel_initial);
                end

                % start next segment from end of current segment
                last_index_1 = extrema_indices(j);

                % for braking calculate backwards from the apex velocity of the segment end
                long_vel = apex_velocity(j);

                for i = extrema_indices(j):-1:last_index_2 % opposite direction from accel

                    % basic kinematics equations
                    lat_accel = long_vel^2*abs(curvature(i));
                    lat_accel_vector_2(i) = lat_accel*sign(curvature(i));
                    long_accel = F_braking(lat_accel,long_vel);
                    if long_vel == obj.car.max_vel
                        long_accel = 0;
                    end
                    long_accel_vector_2(i) = long_accel;
                    long_vel_initial = long_vel;
                    long_vel_vector_2(i) = long_vel_initial;
                    long_vel = sqrt(long_vel^2-2*long_accel*(arclength(i+1)-arclength(i)));
                    % can't exceed max possible velocity for given radius
                    long_vel = min(long_vel,lininterp1(obj.interp_info.radius_vector,obj.interp_info.max_vel_corner_vector,...
                        abs(1/curvature(i)))); 
                    time_2(i) = 2*(arclength(i+1)-arclength(i))/(long_vel+long_vel_initial);
                end

                % end next calculation at end of current segment
                last_index_2 = extrema_indices(j);

                % if apex velocity can not be reached, e.g. segment is too short to
                %   reach apex velocity, then the ending velocity is the velocity
                %   reached during acceleration
                long_vel = min(long_vel_vector_1(end),long_vel_vector_2(end));

            end  
            
            % final longitudinal velocity is minimum of acceleration and braking
            %   velocity profiles
            long_vel_final = min(long_vel_vector_1,long_vel_vector_2);
            indices_2 = find(long_vel_vector_2 == long_vel_final);

            % replace acceleration and time with correct profile
            long_accel_final = long_accel_vector_1;
            long_accel_final(indices_2) = long_accel_vector_2(indices_2);
            long_accel_final = long_accel_final;
            lat_accel_final = lat_accel_vector_1;
            lat_accel_final(indices_2) = lat_accel_vector_2(indices_2);
            time_final = time_1;
            time_final(indices_2) = time_2(indices_2);
            % final time result
            time_final = sum(time_final);
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Autocross(obj)
            arclength = obj.autocross_track(1,:);
            curvature = obj.autocross_track(2,:);
            [long_vel_final,long_accel_final,lat_accel_final,time_final] = ...
                Track_Solver(obj,arclength,curvature);
            obj.times.autocross = time_final*1.03; % scaling factor to reduce endurance points
            time_final
            obj.autocross.time_vec = linspace(0,time_final,100000);
            obj.autocross.long_vel = long_vel_final;
            obj.autocross.long_accel = long_accel_final;
            obj.autocross.lat_accel = lat_accel_final;
        end
        
        function [long_vel_final,long_accel_final,lat_accel_final,time_final] = Endurance(obj)
            arclength = obj.endurance_track(1,:);
            curvature = obj.endurance_track(2,:);
            [long_vel_final,long_accel_final,lat_accel_final,time_final] = ...
                Track_Solver(obj,arclength,curvature);
            time_final = time_final*15; % 15 laps in endurance
            obj.times.endurance = time_final*1.05; % scaling factor due to driver conservatism during enduro
            obj.endurance.time_vec = linspace(0,time_final,100000);
            obj.endurance.long_vel = long_vel_final;
            obj.endurance.long_accel = long_accel_final;
            obj.endurance.lat_accel = lat_accel_final;
        end
        
        function points = computePoints(obj)
            % computes dynamic event points
            % based on 2018/2019 rules
            
            % Acceleration 100 points
            % Skid Pad 75 points
            % Autocross 125 points
            % Efficiency 100 points
            % Endurance 275 points
            
            % B19 points:
            % skidpad: 41.3, accel: 52.7, autocross: 104.9, enduro: 98.8

            % winning time (Michigan 2016, no one faster since)
            skidpad_winning_time = 4.714; 
            
            % winning times (based on 2019 Lincoln)
            accel_winning_time = 4.174;%Michigan 2023
            autocross_winning_time = 45.886;%Michigan 2023
            endurance_winning_time = 1286;%

            % skidpad
            t_your = obj.times.skidpad;
            t_min = min([skidpad_winning_time t_your]);
            t_max = 1.25 * t_min;
            if t_your > t_max
               skidpad_points = 3.5;
            else
               skidpad_points =  71.5 * ((t_max / t_your)^2.0 - 1.0) / ((t_max / t_min)^2.0 - 1.0) + 3.5;
            end


            % accel
            t_your = obj.times.accel;
            t_min = min(accel_winning_time, t_your);
            t_max = t_min * 1.5;
            if t_your > t_max
                accel_points = 4.5;
            else
                accel_points = 95.5 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 4.5;
            end

            % autocross
            t_your = obj.times.autocross;
            t_min = min([autocross_winning_time, t_your]);
            t_max = 1.45 * t_min;
            if t_your > t_max
                autocross_points = 6.5;
            else
                autocross_points = 118.5 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 6.5;
            end

            % endurance
            t_your = obj.times.endurance;
            t_min = min(endurance_winning_time, t_your);
            t_max = 1.45 * t_min;
            if t_your > t_max
                endurance_points = 25;
            else 
                endurance_points = 200 * ((t_max / t_your) - 1.0) / ((t_max / t_min) - 1.0) + 25.0;
            end
            points = struct();
            points.skidpad = skidpad_points;
            points.accel = accel_points;
            points.autocross = autocross_points;
            points.endurance = endurance_points;
            points.total = sum([skidpad_points; 
                                   accel_points;
                                   autocross_points;
                                   endurance_points]);
            obj.points = points;
        end
    end
end

