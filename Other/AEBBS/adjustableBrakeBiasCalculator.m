%close all;
clear all;
load('static brake bias sweep 1.mat');
%load('AEBBS single lapsim run 1.mat')
%carCell = carCell(3,:);
numCars = size(carCell,1);

car = carCell{1};
long_vel_vector = 8:0.5:car.max_vel;%all test velocities
lat_accel = 9.8*0.5;%test lateral acceleration


brake_bias_vector = zeros(numCars,1);
long_decel_matrix = zeros(numCars,numel(long_vel_vector));%row for each car, column for each velocity

for i = 1:numCars
    car = carCell{i,1};
    [F_accel,F_braking] = create_scattered_interpolants2(...
        car.longAccelLookup, car.longDecelLookup);
    brake_bias_vector(i) = car.powertrain.brake_distribution;
    for j = 1:numel(long_vel_vector)
        long_decel_matrix(i,j) = F_braking(lat_accel,long_vel_vector(j));
    end
end

long_decel_matrix = long_decel_matrix/-9.8; % -m/s^2 --> +Gs
set(gca,'linestyleorder',{'-',':','.-'},'colororder',[0 0 1;0 .5 0;1 0 0],'nextplot','add');
%set(gca,'linestyleorder',{'-',':','.-'},'nextplot','add');
hold on;
%plots = plot(long_vel_vector,long_decel_matrix);

plots(1) = plot(long_vel_vector,long_decel_matrix(8,:));
plots(2) = plot(long_vel_vector,max(long_decel_matrix,[],1),'linewidth',4);

%plots(4).LineWidth = 5;
%plots(6).LineWidth = 5;
%plots(8).LineWidth = 5;
%plots(10).LineWidth = 5;


xlabel('Car Velocity (m/s)');
ylabel('Max Braking Acceleration (G)');

title('Max Braking Acceleration vs Car Velocity (Lat Accel = ' + string(lat_accel/9.8) + ' G)');

%ylim([0 inf]);
%ylim([1.2 2]);

legend('Brake Bias: 0.85', 'AEBBS','location','southeast');

%legend(join([repmat("Brake Bias: ",numCars,1) string(brake_bias_vector)])...
%    ,'location','southeast');

%% calculate lookup table

%lookup_table = zeros(4,numel(long_vel_vector));

for i = 1:numel(long_vel_vector)
    [~, index] = max(long_decel_matrix(:,i));
    lookup_table(4,i) = brake_bias_vector(index);
end





