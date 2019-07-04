function [] = plotter(comp,plot_choice)

if plot_choice(1)
    figure
    title('Autocross Track')
    plot(comp.autocross_track(1,:),comp.autocross_track(2,:))
    xlabel('Distance (m)')
    ylabel('Curvature (1/m)')
end

if plot_choice(2)
    figure
    title('Endurance Track')
    plot(comp.endurance_track(1,:),comp.endurance_track(2,:))
    xlabel('Distance (m)')
    ylabel('Curvature (1/m)')
end

if plot_choice(3)
    figure
    plot(comp.interp_info.radius_vector,comp.interp_info.max_vel_corner_vector)
    xlabel('Radius (m)')
    ylabel('Maximum possible velocity')
end

if plot_choice(4)
    figure
    plot(comp.interp_info.long_vel_guess,comp.interp_info.long_accel_matrix/9.81)
    xlabel('Velocity (m/s)')
    ylabel('Maximum possible longitudinal acceleration (g)')
end

if plot_choice(5)
    figure
    plot(comp.accel.time_vec,comp.accel.long_vel_vector)
    title('Accel')
    xlabel('Time (s)')
    ylabel('Velocity (m/s)')
end

if plot_choice(6)
    figure
    plot(comp.accel.time_vec,comp.accel.long_accel_vector)
    title('Accel')
    xlabel('Time (s)')
    ylabel('Longitudinal acceleration (g)')
end

end

