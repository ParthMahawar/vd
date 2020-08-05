
load('drivetrain sweep.mat');

skidpadPoints = zeros(1,11);
accelPoints = zeros(1,11);
autocrossPoints = zeros(1,11);
enduracePoints = zeros(1,11);
totalPoints = zeros(1,11);


for i = 1:11
    skidpadPoints(i) = carCell{i,1}.comp.points.skidpad;
    accelPoints(i) = carCell{i,1}.comp.points.accel;
    autocrossPoints(i) = carCell{i,1}.comp.points.autocross;
    endurancePoints(i) = carCell{i,1}.comp.points.endurance;
    totalPoints(i) = carCell{i,1}.comp.points.total;
end
%%
figure;
plot([35:45]/11, skidpadPoints);
xlabel('Final Drive Ratio');
ylabel('Points');
title('Skidpad');
ylim([50 52]);

figure;
plot([35:45]/11, accelPoints);
xlabel('Final Drive Ratio');
ylabel('Points');
title('Accel');

figure;
plot([35:45]/11, autocrossPoints);
xlabel('Final Drive Ratio');
ylabel('Points');
title('Autocross');
ylim([115 116]);

figure;
plot([35:45]/11, endurancePoints);
xlabel('Final Drive Ratio');
ylabel('Points');
title('Endurance');
ylim([206 208]);

figure;
plot([35:45]/11, totalPoints);
xlabel('Final Drive Ratio');
ylabel('Points');
title('Total');

