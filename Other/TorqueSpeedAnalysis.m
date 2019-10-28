setup_paths

% KTM 350
redline_350 = 13000;
shift_point_350 = 10000;
gears_350 = [2.0, 1.63, 1.33, 1.14, 0.95]; % KTM350
primary_reduction_350 = 3.04; % KTM350
final_drive_350 = 40/11;

% KTM 450
redline_450 = 11500; 
shift_point_450 = 9000; 
gears_450 = [32/16 30/18 28/20 26/22 24/24]; % updated KTM450
primary_reduction_450 = 76/32; % KTM450
final_drive_450 = 40/11;

KTM350_T = KTM350();
KTM450_T = KTM450();
figure
plot(KTM350_T(1,:),KTM350_T(2,:))
hold on
plot(KTM450_T(1,:),KTM450_T(2,:))

figure
for i = 1:5
    gear_ratio = gears_350(i)*primary_reduction_350*final_drive_350;
    plot(KTM350_T(1,:)/gear_ratio,KTM350_T(2,:)*gear_ratio,'k');
    hold on
    gear_ratio = gears_350(i)*primary_reduction_450*final_drive_450;
    plot(KTM450_T(1,:)/gear_ratio,KTM450_T(2,:)*gear_ratio,'b');
end


    

