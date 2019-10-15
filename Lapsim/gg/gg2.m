function paramArr = gg2(car,numWorkers)
% creates velocity-dependent g-g diagram 
% describes max lateral acceleration, max longitudinal acceleration for
%   certain velocity
% inputs car: car object, numWorkers: number of workers for parallelization
% outputs paramArr: array of ParamSets (contains info about each
%   optimization point). 
%   rows: different longitudinal velocities, columns: different lat accels

minV = 5; maxV = car.max_vel-.5;
longVinterval = 1; % m/s
latAgrid = 20;

longVelArr = [minV:longVinterval:maxV maxV];
paramArr(numel(longVelArr),latAgrid) = ParamSet();

% iterate through velocities
parfor (c1 = 1:numel(longVelArr),numWorkers)
    longVel = longVelArr(c1);
    [maxLatx,maxLatLatAccel,maxLatLongAccel,maxLatx0] = max_lat_accel(longVel,car);
    latAccelArr = linspace(0.1,maxLatLatAccel-0.1,latAgrid);
    row = ParamSet();
    row(numel(latAccelArr)) = ParamSet();
        
    % iterate through lateral accelerations
    for c2 = 1:numel(latAccelArr)
        latAccel = latAccelArr(c2);
        [xAccel,longAccel,longAccelx0] = max_long_accel_cornering(longVel,latAccel,car);
        [xBraking,longDecel,brakingDecelx0] = max_braking_decel_cornering(longVel,latAccel,car);       
        carParams = ParamSet(car,longVel); 
        carParams = carParams.setMaxLatParams(maxLatx,maxLatLatAccel,maxLatLongAccel,maxLatx0);
        carParams = carParams.setMaxAccelParams(xAccel,longAccel,latAccel,longAccelx0);
        carParams = carParams.setMaxDecelParams(xBraking,longDecel,latAccel,brakingDecelx0);
        row(c2) = carParams;
    end
    paramArr(c1,:) = row;
end