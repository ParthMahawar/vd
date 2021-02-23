function [normal_load_FL, dist_FL, normal_load_FR, dist_FR, normal_load_RL,dist_RL, normal_load_RR, dist_RR] = calcWheelForcesAndDisplacements(roll_angle, C)
    % + roll angle > car rolls to the left
    
    %The distance the front wheels move as a function of roll angle
    dist_FL = (C.front_width*sind(roll_angle))/2;
    dist_FR = -(C.front_width*sind(roll_angle))/2;

    %The distance the rear wheels move as a function of roll angle
    dist_RR = -(C.rear_width*sind(roll_angle))/2;
    dist_RL = (C.rear_width*sind(roll_angle))/2;

    %Normal Load on front wheels as a function of roll angle
    normal_load_FL = (1 - C.weight_dist) * (C.mass/2) + ...
        roll_angle / C.front_width * ( C.front_spring_stiff + C.front_ARB_stiff );
    normal_load_FR = (1 - C.weight_dist) * (C.mass/2) - ...
        roll_angle / C.front_width * ( C.front_spring_stiff + C.front_ARB_stiff );
    
    %Normal Load on rear wheels as a function of roll angle
    normal_load_RL = (C.weight_dist) * (C.mass/2) + ...
        roll_angle / C.rear_width * ( C.rear_spring_stiff + C.rear_ARB_stiff );
    normal_load_RR = (C.weight_dist) * (C.mass/2) - ...
        roll_angle / C.rear_width * ( C.rear_spring_stiff + C.rear_ARB_stiff );
end







