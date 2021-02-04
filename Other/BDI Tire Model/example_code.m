%% Example Code

%initialize tire
tire = BDI_Tire_Model();

%call returns slip ratio and max force (straight line braking)
normal_force = 120; %newtons

[force, slip_ratio] = peak_longitudinal_force(tire, normal_force);


% showing relation between grip and normal force
force_vector = [];
for F_z = 250:1.5e+03
    [force, ~] = peak_longitudinal_force(tire, F_z);
    force_vector = [force_vector force];
end
plot(250:1.5e+03, force_vector);
xlabel('Normal Force (N)');
ylabel('Peak Longitudinal Force (N)');