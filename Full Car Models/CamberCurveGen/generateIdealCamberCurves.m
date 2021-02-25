%taken from carConfig
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
Fx_parameters = cell2mat(Xbestcell);
load('Fy_combined_parameters_16x10x7.5_LC0.mat'); % F_y combined magic formula parameters
Fy_parameters = cell2mat(Xbestcell);
friction_scaling_factor = 1.05*0.55;
p_i = 12; % tire pressure

%initialize tire object, used for camber evaluation
tire = Tire2(0,p_i,Fx_parameters,Fy_parameters,friction_scaling_factor);

%Vehicle Width in inches
C.front_width = 47;
C.rear_width = 47;

%Spring Roll Stiffness
C.front_spring_stiff = 2860; %numbers from LLTD Doc
C.rear_spring_stiff = 2911;

%ARB Roll Stiffness (Currently on Short)
C.front_ARB_stiff = 0;
C.rear_ARB_stiff = 1493;

%Car Weight Distribution
C.weight_dist = 0.54; % percentage of weight in rear

%Total Car Weight
C.mass = 395; % not including driver (lb)

%% camber curve optimization
%roll angles and cambers to search through
roll_angle_vector = 0:0.05:1;
camber_vector = -5:0.05:3;

%arrays to store ideal camber curves 
dist_FL_vector = zeros(1,numel(roll_angle_vector));
ideal_FL_camber_vector = zeros(1,numel(roll_angle_vector));

dist_FR_vector = zeros(numel(roll_angle_vector));
ideal_FR_camber_vector = zeros(1,numel(roll_angle_vector));

dist_RL_vector = zeros(numel(roll_angle_vector));
ideal_RL_camber_vector = zeros(1,numel(roll_angle_vector));

dist_RR_vector = zeros(numel(roll_angle_vector));
ideal_RR_camber_vector = zeros(1,numel(roll_angle_vector));

max_Fy_front_vector = zeros(1,numel(roll_angle_vector));
max_Fy_rear_vector = zeros(1,numel(roll_angle_vector));


%Loop through Roll Angles for front axle
for i = 1:numel(roll_angle_vector)
    
    %car roll angle
    roll_angle = roll_angle_vector(i)
    
    %calculate normal loads and wheel displacements at each wheel
    [normal_load_FL, dist_FL, normal_load_FR, dist_FR, normal_load_RL,dist_RL, normal_load_RR, dist_RR] =...
        calcWheelForcesAndDisplacements(roll_angle, C);
    
    dist_FL_vector(i)= dist_FL;
    dist_FR_vector(i)= dist_FR;
    dist_RL_vector(i)= dist_RL;
    dist_RR_vector(i)= dist_RR;
    
    %keep track of best lateral force and cambers
    ideal_gamma_L=0;
    ideal_gamma_R=0;
    max_F_y_tot=0;
    
    %front axle, search through camber combinations for ideal combination
    for gamma_L = camber_vector
        for gamma_R = camber_vector
            [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(normal_load_FL, normal_load_FR, -gamma_L, gamma_R, tire);
            
            F_y_L;
            F_y_R;
            
            % if new configuration develops more lateral force, replace stored values
            if max_F_y_tot < F_y_tot
                ideal_gamma_L = gamma_L;
                ideal_gamma_R = gamma_R;
                max_F_y_tot = F_y_tot;
            end
        end
    end
    %store ideal camber for front tires
    ideal_FL_camber_vector(i) = ideal_gamma_L;
    ideal_FR_camber_vector(i) = ideal_gamma_R;
    max_Fy_front_vector(i) = F_y_tot;
    
    %front axle, search through camber combinations for ideal combination
    for gamma_L = camber_vector
        for gamma_R = camber_vector
            [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(normal_load_RL, normal_load_RR, -gamma_L, gamma_R, tire);
            
            F_y_L;
            F_y_R;
            
            % if new configuration develops more lateral force, replace stored values
            if max_F_y_tot < F_y_tot
                ideal_gamma_L = gamma_L;
                ideal_gamma_R = gamma_R;
                max_F_y_tot = F_y_tot;
            end
        end
    end
    %store ideal camber for front tires
    ideal_RL_camber_vector(i) = ideal_gamma_L;
    ideal_RR_camber_vector(i) = ideal_gamma_R;
    max_Fy_rear_vector(i) = F_y_tot;
end

%% plotting by roll
subplot(3,1,1)
plot([-flip(roll_angle_vector) roll_angle_vector], [flip(ideal_FL_camber_vector) ideal_FR_camber_vector]);
title('Front Camber Curve');
xlabel('roll angle');
ylabel('ideal camber');

subplot(3,1,2)
plot([-flip(roll_angle_vector) roll_angle_vector], [flip(ideal_RL_camber_vector) ideal_RR_camber_vector]);
title('Rear Camber Curve');
xlabel('roll angle');
ylabel('ideal camber');

subplot(3,1,3)
plot(roll_angle_vector, max_Fy_front_vector);
hold on;
plot(roll_angle_vector, max_Fy_rear_vector);
legend('front_Fy', 'rear Fy');
title('Lateral Force');
xlabel('roll angle');
ylabel('max_lateral_force');


%% compare to existing

existing_camber_FL_vector = 
existing_camber_FR_vector = 
existing_camber_RL_vector = 
existing_camber_RR_vector = 

existing_Fy_front_vector = zeros(1,numel(roll_angle_vector));
existing_Fy_rear_vector = zeros(1,numel(roll_angle_vector));

for i = 1:numel(roll_angle_vector)
    %car roll angle
    roll_angle = roll_angle_vector(i)
    
    %calculate normal loads and wheel displacements at each wheel
    [normal_load_FL, ~, normal_load_FR, ~, normal_load_RL, ~, normal_load_RR, ~] =...
        calcWheelForcesAndDisplacements(roll_angle, C);
    
    
    [existing_Fy_front_vector(i), ~, ~, ~, ~, ~] = ...
        singleAxleCamberEvaluation(normal_load_FL, normal_load_FR, -existing_camber_FL_vector(i), existing_camber_FR_vector(i), tire);
    
    [existing_Fy_rear_vector(i), ~, ~, ~, ~, ~] = ...
        singleAxleCamberEvaluation(normal_load_RL, normal_load_RR, -existing_camber_RL_vector(i), existing_camber_RR_vector(i), tire);
end





%% plotting by roll
% subplot(2,2,1)
% plot(roll_angle_vector, ideal_FL_camber_array);
% title('Front Left Camber Curve');
% 
% subplot(2,2,2)
% plot(roll_angle_vector, ideal_FR_camber_array);
% title('Front Right Camber Curve');
% 
% subplot(2,2,3)
% plot(roll_angle_vector, ideal_RL_camber_array);
% title('Rear Left Camber Curve');
% 
% subplot(2,2,4)
% plot(roll_angle_vector, ideal_RR_camber_array);
% title('Rear Right Camber Curve');

%% plotting displacement
% 
% subplot(2,2,1)
% plot(dist_FL_array, ideal_FL_camber_array);
% title('Front Left Camber Curve');
% 
% subplot(2,2,2)
% plot(dist_FR_array, ideal_FR_camber_array);
% title('Front Right Camber Curve');
% 
% subplot(2,2,3)
% plot(dist_RL_array, ideal_RL_camber_array);
% title('Rear Left Camber Curve');
% 
% subplot(2,2,4)
% plot(dist_RR_array, ideal_RR_camber_array);
% title('Rear Right Camber Curve');



