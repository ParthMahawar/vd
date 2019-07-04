% Lapsim2
% Main steady-state lapsim script. This is traditional lapsim. It finds the
% g-g diagram over different velocities (max lateral and longitudinal
% acceleration at a given velocity), then uses those to predict performance
% in the dynamic events.
% HOW TO USE:
% 1) carConfig.m: define the car you want to test
% 2) run this script
% 3) results are in the comp object, which is stored in the corresponding
% car object. Index into carCell to get the car you want, then open the
% comp object.
clear
setup_paths
carCell = carConfig(); %generate all cars to sim over
numCars = size(carCell,1);
time = struct();time.prev = 0; time.curr = 0;
tic
% Set numWorkers to number of cores for better performance
numWorkers = 0;
if numWorkers ~= 0
    disp('The parallel toolbox takes a few minutes to start.')
    disp('Set numWorkers to 0 for single-car runs')
end

for i = 1:numCars
    car = carCell{i,1};
    accelCar = carCell{i,2};
    fprintf("car %d of %d - starting g-g\n",[i numCars]);
    paramArr = gg2(car,numWorkers);
    fprintf("car %d of %d - g-g complete\n",[i numCars]);
    time.curr = floor(toc);
    fprintf("Stage Time: %d s; Total time elapsed: %d s\n",[time.curr-time.prev time.curr]);
    time.prev = time.curr;
    car = makeGG(paramArr,car); %post-processes gg data and stores in car
    comp = Events2(car,accelCar); 
    comp.calcTimes();       %run events and calc points
    car.comp = comp;        %store in array
    carCell{i,1} = car; %put updated car back into array. Matlab is pass by value, not pass by reference
    fprintf("car %d of %d - points calculated\n",[i numCars]);
    time.curr = floor(toc);
    fprintf("Stage Time: %d s; Total time elapsed: %d s\n",[time.curr-time.prev time.curr]);
end
fprintf("done\n");

%% Saving
%save('blahblah.mat','carCell');




%% Car Plotting
% 
% % set desired plots to 1
% plot1 = 1; % velocity-dependent g-g diagram scatter plot
% plot2 = 0; % velocity-dependent g-g diagram surface
% plot3 = 0; % max accel for given velocity and lateral g
% plot4 = 0; % max braking for given velocity and lateral g
% plot5 = 0; % scattered interpolant version of plot3
% plot6 = 0; % scattered interpolant version of plot4
% plot7 = 0; % 2D g-g diagram for velocity specified below
% 
% g_g_vel = 10; % can input vector to overlay different velocities
% 
% plot_choice = [plot1 plot2 plot3 plot4 plot5 plot6 plot7];
% 
% car.ggPoints(:,1),car.ggPoints(:,2),car.ggPoints(:,3)
% 
% plotter(car,g_g_vel,plot_choice);

% 
% %% Event Plotting
% 
% figure
% title('Autocross Track')
% plot(comp.autocross_track(1,:),comp.autocross_track(2,:))
% xlabel('Distance (m)')
% ylabel('Curvature (1/m)')
% 
% figure
% title('Endurance Track')
% plot(comp.endurance_track(1,:),comp.endurance_track(2,:))
% xlabel('Distance (m)')
% ylabel('Curvature (1/m)')
% 
% figure
% plot(comp.interp_info.radius_vector,comp.interp_info.max_vel_corner_vector)
% xlabel('Radius (m)')
% ylabel('Maximum possible velocity')
% 
% figure
% plot(comp.interp_info.long_vel_guess,comp.interp_info.long_accel_matrix/9.81)
% xlabel('Velocity (m/s)')
% ylabel('Maximum possible longitudinal acceleration (g)')
% 
