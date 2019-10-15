%% B19
Kf = 3243; % in-lb
Kr = 8402; % in-lb
MR = 0.75; 

spring_travel = 0.040; % spring compression on one side with max steer
wheel_travel = spring_travel/MR;
track_width = 50; % in

% roll angle caused by wheel lift from caster at max steer (deg)
roll_caster = atand(2*spring_travel/track_width);

weight_transfer = (Kf*Kr)/(Kf+Kr)*roll_caster/track_width % lbs
%weight_transfer2 = Kf*(roll_caster-Kf/(Kf+Kr)*roll_caster)/track_width;

%% B20
Kf = 2860; % in-lb
Kr = 5582; % in-lb
MR = 0.75; 

spring_travel = 0.075; % spring compression on one side with max steer
wheel_travel = spring_travel/MR;
track_width = 47; % in

% roll angle caused by wheel lift from caster at max steer (deg)
roll_caster = atand(2*spring_travel/track_width);

weight_transfer = (Kf*Kr)/(Kf+Kr)*roll_caster/track_width % lbs
%weight_transfer2 = Kf*(roll_caster-Kf/(Kf+Kr)*roll_caster)/track_width;

%%

spring_travel_vec = linspace(0,0.3,1000);

for i = 1:numel(spring_travel_vec)
    spring_travel = spring_travel_vec(i);
    wheel_travel = spring_travel/MR;
    track_width = 47; % in

    % roll angle caused by wheel lift from caster at max steer (deg)
    roll_caster = atand(2*spring_travel/track_width);
    
    Kf = 5182; % in-lb
    Kr = 7774; % in-lb

    weight_transfer = (Kf*Kr)/(Kf+Kr)*roll_caster/track_width; % lbs
    
    LLTD = 0.5;
    weight = 560; % lb
    cornering_g = 1.5;
    cg_z = 12; % in
    
    caster_weight_transfer = weight_transfer; % lb
    
    total_weight_transfer = cornering_g*cg_z*weight/track_width;
    front_weight_transfer = LLTD*total_weight_transfer;
    rear_weight_transfer = (1-LLTD)*total_weight_transfer;
    
    front_weight_transfer_withcaster = LLTD*total_weight_transfer-caster_weight_transfer;
    rear_weight_transfer_withcaster = (1-LLTD)*total_weight_transfer+caster_weight_transfer;
    LLTD_new = front_weight_transfer_withcaster/total_weight_transfer;
    
    deltaLLTD(i) = LLTD-LLTD_new;
end

plot(spring_travel_vec,deltaLLTD);

%% B18
Kf = 5182; % in-lb
Kr = 7774; % in-lb

spring_travel = 0.080; % spring compression on one side with max steer
track_width = 50; % in

% roll angle caused by wheel lift from caster at max steer (deg)
roll_caster = atand(2*spring_travel/track_width);

weight_transfer = (Kf*Kr)/(Kf+Kr)*roll_caster/track_width % lbs

%% B19 with flexy chassis

Kf = 3243; % in-lb
Kr = 8402; % in-lb

Kc = 2000*12; % in-lb

spring_travel = 0.040; % spring compression on one side with max steer
track_width = 50; % in

% roll angle caused by wheel lift from caster at max steer (deg)
roll_caster = atand(2*spring_travel/track_width);

roll_angle = Kf/(Kf+((Kr*Kc)/(Kr+Kc)))*roll_caster; % deg
weight_transfer2 = Kf*(roll_caster-roll_angle)/track_width;

%% Weight transfer calculations

LLTD = 0.5;
weight = 560; % lb
cornering_g = 1.5;
cg_z = 12; % in
track_width = 50; % in

caster_weight_transfer = 30; % lb

total_weight_transfer = cornering_g*cg_z*weight/track_width;
front_weight_transfer = LLTD*total_weight_transfer;
rear_weight_transfer = (1-LLTD)*total_weight_transfer;

front_weight_transfer_withcaster = LLTD*total_weight_transfer-caster_weight_transfer;
rear_weight_transfer_withcaster = (1-LLTD)*total_weight_transfer+caster_weight_transfer;
LLTD_new = front_weight_transfer_withcaster/total_weight_transfer;

LLTD-LLTD_new





