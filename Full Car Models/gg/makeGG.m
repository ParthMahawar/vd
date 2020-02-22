function car = makeGG(paramArr,car)
ggpoints = [];
ss_info = [];
accel_info = [];
decel_info = [];
longAccelLookup = []; 
longDecelLookup = []; 
arr = reshape(paramArr,[numel(paramArr) 1]);
for i = 1:numel(arr)
    pSet = arr(i);
    p1 = []; p2 = []; p3 = []; p4 = []; x0 = []; x1 = []; x2 = [];
    
    if pSet.maxLatFlag == 1
        x0 = pSet.maxLatx;
    end   
    
    %x: long %y: lat %z: velo
    if pSet.maxLatFlag == 1 && pSet.maxLongFlag == 1
        p1 = [pSet.maxLongLongAccel pSet.maxLongLatAccel pSet.longVel];
        p2 = [pSet.maxLongLongAccel -pSet.maxLongLatAccel pSet.longVel];
        x1 = pSet.maxLongAccelx;
    end
    
    if pSet.maxLatFlag == 1 && pSet.maxBrakeFlag == 1
        p3 = [pSet.maxBrakeLongDecel pSet.maxBrakeLatAccel pSet.longVel];
        p4 = [pSet.maxBrakeLongDecel -pSet.maxBrakeLatAccel pSet.longVel];
        x2 = pSet.maxBrakeDecelx;
    end
    
    ggpoints = [ggpoints; p1; p2; p3; p4];
    ss_info = [ss_info; x0];
    accel_info = [accel_info; x1];
    decel_info = [decel_info; x2];
    longAccelLookup = [longAccelLookup; p1];
    longDecelLookup = [longDecelLookup; p3];
end

car.ggPoints = ggpoints;
car.ss_info = ss_info;
car.accel_info = accel_info;
car.decel_info = decel_info;
car.longAccelLookup = longAccelLookup;
car.longDecelLookup = longDecelLookup;
