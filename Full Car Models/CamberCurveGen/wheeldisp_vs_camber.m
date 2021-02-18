%taken from carConfig
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
Fx_parameters = cell2mat(Xbestcell);
load('Fy_combined_parameters_16x10x7.5_LC0.mat'); % F_y combined magic formula parameters
Fy_parameters = cell2mat(Xbestcell);
friction_scaling_factor = 1.05*0.55;
p_i = 12; % pressure

tire = Tire2(0,p_i,Fx_parameters,Fy_parameters,friction_scaling_factor);

%Vehicle Width in inches
C.front_width= 47;
C.rear_width= 47;

%Spring Roll Stiffness
C.front_spring_stiff= 2860;
C.rear_spring_stiff=2911;

%ARB Roll Stiffness (Currently on Short)
C.front_ARB_stiff=0;
C.rear_ARB_stiff=1493;

%Car Weight Distribution
C.weight_dist = 0.54; % percentage of weight in rear

%Total Car Weight
C.mass = 179.2 + 5.9; % not including driver (395 lb)

roll_angle_vector=0:1:10;
camber_vector=-5:1:5;

dist_FL_array = zeros(1,numel(roll_angle_vector));
ideal_FLcamber_array= zeros(1,numel(roll_angle_vector));

dist_FR_array = zeros(numel(roll_angle_vector));
ideal_FRcamber_array= zeros(1,numel(roll_angle_vector));

dist_RL_array = zeros(numel(roll_angle_vector));
ideal_RLcamber_array= zeros(1,numel(roll_angle_vector));

dist_RR_array = zeros(numel(roll_angle_vector));
ideal_RRcamber_array= zeros(1,numel(roll_angle_vector));



%Loop through Roll Angles for front axle
for i= 1:numel(roll_angle_vector)
    roll_angle= roll_angle_vector(i);
    
    [normal_load_FL, dist_FL, normal_load_FR, dist_FR, normal_load_RL,dist_RL, normal_load_RR, dist_RR]= CamberCurve(roll_angle, C);
     dist_FL_array(i)= dist_FL;
     dist_FR_array(i)= dist_FR;
    
    ideal_gamma_L=0;
    ideal_gamma_R=0;
    max_F_y_tot=0;
    
    for gamma_L = camber_vector
            for gamma_R= camber_vector
                [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(normal_load_FL, normal_load_FR, -gamma_L, gamma_R, tire);
               
                F_y_tot
                
                if max_F_y_tot < F_y_tot
                    ideal_gamma_L= gamma_L;
                    ideal_gamma_R= gamma_R;
                    max_F_y_to=F_y_tot;
                end
                
            end 
    end
     ideal_FLcamber_array(i)=ideal_gamma_L;  
     ideal_FRcamber_array(i)=ideal_gamma_R;
end


%Loop through Roll Angles for rear axle
for i= 1:numel(roll_angle_vector)
    roll_angle= roll_angle_vector(i);
    
    [normal_load_FL, dist_FL, normal_load_FR, dist_FR, normal_load_RL,dist_RL, normal_load_RR, dist_RR]= CamberCurve(roll_angle, C);
    dist_RL_array(i)= dist_RL;
    dist_RR_array(i)= dist_RR;
    
    ideal_gamma_L=0;
    ideal_gamma_R=0;
    max_F_y_tot=0;
    
    for gamma_L = camber_vector
            for gamma_R= camber_vector
                [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(normal_load_RL, normal_load_RR, -gamma_L, gamma_R, tire);
                
                if max_F_y_tot < F_y_tot
                    ideal_gamma_L= gamma_L;
                    ideal_gamma_R= gamma_R;
                    max_F_y_to=F_y_tot;
                end
                
            end 
    end
     ideal_RLcamber_array(i)=ideal_gamma_L;  
     ideal_RRcamber_array(i)=ideal_gamma_R;
end


