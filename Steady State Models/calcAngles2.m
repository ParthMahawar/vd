function [y_new, forces, nextFz] = calcAngles2(car,x,y0,forces,xdot0)

n = round(car.TSmpc/car.TSdyn,0);
dt = car.TSdyn;
y = y0;

cArr = zeros(1,4); %intialize damp coeff array for all tires
MRArr = ones(1,4); %intialize MR array for all tires

% tires 3 and 4 switched from jazar model
for i = 2:n+1      
    % calculate mass, damping, stiffness matrices
    [m,c,k] = calc_MCK(car,y(4:7),y(11:14));
    
    tireForceXY = forces.Ftires;
    tireForceXY(:,3) = 0;
    yawRate = x(2);
    longVel = x(3);
    latVel = x(4);
    
    lat_accel = longVel.*yawRate+xdot0(4,:);
    long_accel = -latVel.*yawRate+xdot0(3,:);
    rollMoment = -car.M*lat_accel*(car.h_g-car.h_rc);
    pitchMoment = -car.M*long_accel*(car.h_g-car.h_rc);
        
    a_1 = car.l_f;
    a_2 = car.l_r;
    b_1 = car.t_f/2;
    b_2 = car.t_f/2;
    jacking_rotation = [1 1 1 1; b_1 -b_2 b_1 -b_2; -a_1 -a_1 a_2 a_2];
    jacking_Fz = [car.h_rf/b_1; -car.h_rf/b_2; car.h_rr/b_1; -car.h_rr/b_2].*tireForceXY(:,2);
    % jacking_vec: [vertical force; roll moment; pitch moment]
    % ISO convention
    jacking_vec = jacking_rotation*jacking_Fz;
    
    %add up all applied moments, using given position vectors
    sumM = 0;
    for i = 1:size(forces.F,1)
        sumM = sumM+cross(forces.F(i,4:6),forces.F(i,1:3)); 
    end
    rollMoment = rollMoment+sumM(1);
    pitchMoment = pitchMoment-sumM(2);
    
    % transform from SAE coordinate system to ISO
    % [applied sprung mass vertical force, roll moment, pitch moment,
    % unsprung mass forces]
    F = [-sum(forces.F(:,3)) rollMoment pitchMoment 0 0 0 0]';
    F = F + [jacking_vec; jacking_Fz];

    % state vector: [x phi theta x1 x2 x3 x4]'
    yd = [y(8:end); m\(F-c*y(8:end)-k*y(1:7))];
    
    nextFz = -diag([car.k_tf car.k_tf car.k_tr car.k_tr])*y(4:7);
    y = y+yd*car.TSdyn;
end

y_new = y;

end