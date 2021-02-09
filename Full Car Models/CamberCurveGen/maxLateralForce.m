function [F_y_tot, F_y_L, F_y_L, M_x_L, M_x_R, alpha_val] = (normal_load_L, normal_load_R, gamma_L, gamma_R, tire)

Fx_parameters = cell2mat(load('Fx_combined_parameters_run38_30.mat')); % F_x combined magic formula parameters
    left_tire = tire;
    right_tire = tire;
    left_tire.gamma = gamma_L;
    right_tire.gamma = gamma_R;
    
%     load('Lapsim_Fy_combined_parameters_1965run15.mat'); % F_y combined magic formula parameters
%     tireParams.Fy_parameters = cell2mat(Xbestcell);
%     tireParams.friction_scaling_factor = 1.05*0.55; % scales tire forces to account for test/road surface difference
% 
%     left_tire = Tire2(gamma,p_i,Fx_parameters,Fy_parameters,friction_scaling_factor)
%     
%     tire_lat_force = F_y(obj,alpha,kappa,F_z)

    for alpha = -15:1:15
        F_y_from_left_tire = F_y(obj,alpha,kappa,F_z)
end