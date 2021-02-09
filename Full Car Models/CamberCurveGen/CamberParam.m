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



% %Springrates in lbs/in
% C.front_spring_rate= 350;
% C.rear_spring_rate= 200;
% 
% %Motion Ratio
% C.front_MR=.75;
% C.rear_MR=1;
% 
% %ARB Rates
% C.front_ARB_rate=0;
% C.rear_ARB_rate=0.25;
% 
% %ARB Motion Ratio
% C.front_ARB_mr
% C.rear_ARB_mr
