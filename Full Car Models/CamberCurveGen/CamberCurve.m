function[normal_load_FL, dist_FL, normal_load_FR, dist_FR, normal_load_RL,dist_RL, normal_load_RR, dist_RR]= CamberCurve(theta, C)

% %The distance the front wheels moves as a function of roll angle
 dist_FL = (C.front_width*sind(theta))/2;
 dist_FR = -(C.front_width*sind(theta))/2;

%The distance the rear wheels move as a function of roll angle
dist_RL = (C.rear_width*sind(theta))/2;
dist_RR = -(C.rear_width*sind(theta))/2;
 
%Normal Load on rear wheels as a function of roll angle
normal_load_RL = theta*((C.rear_spring_stiff)*(C.rear_width/2)+((C.rear_ARB_stiff)/(C.rear_width/2)))+(carParams.weight_dist)*(carParams.mass/2);
normal_load_RR = -theta*((C.rear_spring_stiff)*(C.rear_width/2)+((C.rear_ARB_stiff)/(C.rear_width/2)))+(carParams.weight_dist)*(carParams.mass/2);

%Normal Load on front wheels as a function of roll angle
normal_load_FL = theta*((C.front_spring_stiff)*(C.front_width/2)+((C.front_ARB_stiff)/(C.front_width/2)))+(C.front_weight_distr)*(carParams.mass/2);
normal_load_FR = -theta*((C.front_spring_stiff)*(C.front_width/2)+((C.front_ARB_stiff)/(C.front_width/2)))+(C.front_weight_distr)*(carParams.mass/2);

% %difference betwwen left and right ride height
% front_ARB_dist= dist_FL- dist_FR;
% rear_ARB_dist= dist_RL- dist_RR;
% 
% %Wheelrate
% front_wheelrate = C.front_spring_rate*(C.front_MR)^2;
% rear_wheelrate = C.rear_spring_rate*(C*rear_MR)^2;
% 
% %Front Springs Forces
% normal_load_FL= front_wheelrate*dist_FL + C.front_ARB_rate*front_ARB_dist;
% normal_load_FR= front_wheelrate*dist_FR + C.front_ARB_rate*front_ARB_dist;
% 
% %Rear Springs Forces
% normal_load_RL= rear_wheelrate* dist_RL+ C.rear_ARB_rate * rear_ARB_dist + C.ARB_roll_stifness*C.rear_width;
% normal_load_RR= rear_wheelrate* dist_RL+ C.rear_ARB_rate * rear_ARB_dist;

end







