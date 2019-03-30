function outputs = fullDynamics(car,uArr,x0,n)

% state vectors
FzArr = zeros(4,n); % vertical forces on tires
xArr = zeros(14,n); % state array
yArr = zeros(14,n);
xArr(:,1) = x0;
dt = car.TSmpc;
g = 9.81;

% applied forces (SAE coordinate system)
FapTotal = cell(n,1); %applied forces at each step
Fxyz = [0 0 0]; % applied force
Rxyz = [0 0 car.h_g]; % position of applied force
Fg = [0 0 car.M*g]; % gravity
Rg = [0 0 car.h_g];
Fconstant = [Fg Rg]; 

for i = 1:n
    forces = struct();
    forces.T = 0; % wheel torques
    forces.F = Fconstant; % applied forces
    forces.Ftires = zeros(4,6); % forces applied by tires
    forces.Fxw = 0;         % x forces in front wheels tire csys
    forces.Fx = 0;          % x forces, in car coordinate system, kept for compatibility
    FapTotal{i} = forces;
end
 
for i = 2:n  
    % state vector y: [x phi theta x1 x2 x3 x4 x' phi' theta' x1' x2' x3' x4']'
    y = yArr(:,i-1);
    forces1 = FapTotal{i-1};
    x = xArr(:,i-1);
    u = uArr(:,i-1);
    
    % add powertrain and aero forces
    [forces2, Gr] = car.calcForces(x,u,forces1);
    
    % add tire forces
    forces3 = car.calcTireForces(x,u,forces2);
       
    % calculates new roll and pitch angles
    % calculates new normal forces
    %[outputs,forces4,nextFz] = calcAngles(car,x,angles,forces3);    
    [y_new,forces4,nextFz] = calcAngles2(car,x,y,forces3);    

    % applies forces.F to the car to produce xdot
    [xdot, forces5] = car.dynamics(x,forces4,Gr);

    %advance state
    xArr(:,i) = xArr(:,i-1) + dt*xdot; 
    
    yArr(:,i) = y_new;
    FzArr(:,i) = forces5.Ftires(:,3);
    
    %store all applied forces
    FapTotal{i-1} = forces5; 
    nextF = FapTotal{i};
    nextF.Ftires(:,3) = nextFz;
    FapTotal{i} = nextF;
end

outputs = struct();
outputs.xArr = xArr;
outputs.FzArr = FzArr;
outputs.yArr = yArr;