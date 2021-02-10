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
C.front_weight_distr=0.52;
carParams.weight_dist = 0.54; % percentage of weight in rear

%Total Car Weight
carParams.mass = 179.2 + 5.9; % not including driver (395 lb)

dist_FL_array = zeros(1,numel());
ideal_FLcamber_array= zeros(1,numel());

dist_FR_array = zeros(numel());
ideal_FRcamber_array= zeros(1,numel());

dist_RL_array = zeros(numel());
ideal_RLcamber_array= zeros(1,numel());

dist_RR_array = zeros(numel());
ideal_RRcamber_array= zeros(1,numel());


%Loop through Roll Angles for front axle
for roll_angle = 0:1:10
    %The distance the front wheels moves as a function of roll angle
    dist_FL = (C.front_width*sind(roll_angle))/2;
    dist_FL_array(roll_angle) = dist_FL;
    
    %Normal Load on front wheels as a function of roll angle
    normal_load_FL = roll_angle*((C.front_spring_stiff)*(C.front_width/2)+((C.front_ARB_stiff)/(C.front_width/2)))+(C.front_weight_distr)*(carParams.mass/2);
    normal_load_FL_array(roll_angle)= normal_load_FL;
    
    
    for gamma_L = -10:1:10
        [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire);
        
        
        for roll_angle = -10:1:10
            %The distance the front wheels moves as a function of roll angle
            dist_FR = -(C.front_width*sind(roll_angle))/2;
            dist_FR_array(roll_angle) = dist_FR;
            
            %Normal Load on front wheels as a function of roll angle
            normal_load_FR = -roll_angle*((C.front_spring_stiff)*(C.front_width/2)+((C.front_ARB_stiff)/(C.front_width/2)))+(C.front_weight_distr)*(carParams.mass/2);
            normal_load_FR_array(roll_angle)=normal_load_FR;
            
            for gamma_R= -10:1:10
                [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire);
            end
        end
    end
end



%Loop through Roll Angles for rear axle
for roll_angle= 0:1:10
    %The distance the rear wheels moves as a function of roll angle
    dist_RL = (C.rear_width*sind(roll_angle))/2;
    dist_RL_array(roll_angle)= dist_RL;
    
    %Normal Load on front wheels as a function of roll angle
    normal_load_RL = roll_angle*((C.rear_spring_stiff)*(C.rear_width/2)+((C.rear_ARB_stiff)/(C.rear_width/2)))+(carParams.weight_dist)*(carParams.mass/2);
    normal_load_RL_array(roll_angle)= normal_load_RL;
    
    
    for gamma_L= -10:1:10
        [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire);
        
        
        for roll_angle=0:1:10
            %The distance the rear wheels move as a function of roll angle
            dist_RR = -(C.rear_width*sind(roll_angle))/2;
            dist_RR_array(roll_angle)= dist_RR;
            
            %Normal Load on rear wheels as a function of roll angle
            normal_load_RR = -roll_angle*((C.rear_spring_stiff)*(C.rear_width/2)+((C.rear_ARB_stiff)/(C.rear_width/2)))+(carParams.weight_dist)*(carParams.mass/2);
            normal_load_RR_array(roll_angle)= normal_load_RR;
            
            for gamma_R= -10:1:10
                [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire);
            end
        end
    end
end
    
    
    function [F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire)
    %for negative camber, gamma_L -> + & gamma_R -> -
    left_tire = tire;
    right_tire = tire;
    left_tire.gamma = gamma_L;
    right_tire.gamma = gamma_R;
    
    left_tire.gamma
    
    % sweep through slip angles to find maximum lateral force for the
    % entire axle
    F_y_tot = 0;
    F_y_L = 0;
    F_y_R = 0;
    alpha_max = 0;
    for alpha = -(0:1:15)
        kappa = 0;
        % turning right - for sign convention | F_y -> + | alpha -> -
        F_y_L_temp = F_y(left_tire, alpha, kappa, F_z_L);
        F_y_R_temp = F_y(right_tire, alpha, kappa, F_z_R);
        if F_y_L_temp + F_y_R_temp > F_y_tot
            F_y_L = F_y_L_temp;
            F_y_R = F_y_R_temp;
            F_y_tot = F_y_L + F_y_R;
        end
    end
    alpha_val = alpha_max;
    
    %still have to add overturning moment
    M_x_L = 0;
    M_x_R = 0;
    end
