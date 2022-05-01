function outputs = fullDynamics(car,uArr,x0,n)

% state vectors
FzArr = zeros(4,n); % vertical forces on tires

% state array x (vector of 14 values)
xArr = zeros(14,n); 
xdotArr = zeros(14,n);
% 1: yaw angle 2: yaw rate 3: long velocity 4: lat velocity
% 5: x position of cg 6: y position of cg
% 7:  FL angular position 8:  FL angular velocity
% 9:  FR angular position 10: FR angular velocity
% 11: RL angular position 12: RL angular velocity
% 13: RR angular position 14: RR angular velocity


yArr = zeros(14,n); % state array y: [x phi theta x1 x2 x3 x4 x' phi' theta' x1' x2' x3' x4']'
% 1: x - bounce 
% 2: phi - roll angle
% 3: theta - pitch (?)
% 4: x1 - FL Wheel Position
% 5: x2 - FR Wheel Position
% 7: x3 - RL Wheel Position
% 7: x4 - RR Wheel Position
xArr(:,1) = x0;
dt = car.TSmpc;
g = 9.81;

% applied forces (SAE coordinate system)
% x forwards, y right, z downwards
% origin at cg location at average roll center height
Fapplied = cell(n,1); %applied forces at each step
Fxyz = [0 0 0]; % applied force
Rxyz = [0 0 0]; % position of applied force
Fg = [0 0 car.M*g]; % gravity
Rg = [0 0 0];
Fconstant = [Fg Rg]; 

for i = 1:n
    forces = struct();
    forces.T = 0; % wheel torques
    forces.F = Fconstant; % applied forces
    forces.Ftires = zeros(4,6); % forces applied by tires
    forces.Fxw = 0;         % x forces in front wheels tire csys
    forces.Fx = 0;          % x forces, in car coordinate system, kept for compatibility
    Fapplied{i} = forces;
end

for i = 2:n  
    % state vector y: [x phi theta x1 x2 x3 x4 x' phi' theta' x1' x2' x3' x4']'
    y = yArr(:,i-1);
    forces1 = Fapplied{i-1};
    x = xArr(:,i-1);
    xdot0 = xdotArr(:,i-1);
    u = uArr(:,i-1);
    
    % add powertrain and aero forces
    [forces2, Gr] = car.calcForces(x,u,forces1);
    
    % add tire forces
    forces3 = car.calcTireForces(x,u,forces2);
       
    % calculates new roll and pitch angles
    % calculates new normal forces
    %[outputs,forces4,nextFz] = calcAngles(car,x,angles,forces3);    
    [y_new,forces4,nextFz] = calcAngles2(car,x,y,forces3,xdot0);    

    % applies forces.F to the car to produce xdot
    [xdot, forces5] = car.dynamics(x,forces4,Gr);

    xdotArr(:,i) = xdot;
    %advance state
    xArr(:,i) = xArr(:,i-1) + dt*xdot; 
    
    yArr(:,i) = y_new;
    FzArr(:,i) = forces5.Ftires(:,3);
    
    %store all applied forces
    Fapplied{i-1} = forces5; 
    nextF = Fapplied{i};
    nextF.Ftires(:,3) = nextFz;
    Fapplied{i} = nextF;
end

outputs = struct();
outputs.xArr = xArr;
outputs.xdotArr = xdotArr;
outputs.FzArr = FzArr;
outputs.yArr = yArr;