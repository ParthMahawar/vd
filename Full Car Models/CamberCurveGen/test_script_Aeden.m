%taken from carConfig
load('Fx_combined_parameters_run38_30.mat'); % F_x combined magic formula parameters
Fx_parameters = cell2mat(Xbestcell);
load('Lapsim_Fy_combined_parameters_1965run15.mat'); % F_y combined magic formula parameters
Fy_parameters = cell2mat(Xbestcell);
friction_scaling_factor = 1.05*0.55;
p_i = 12; % pressure

tire = Tire2(0,p_i,Fx_parameters,Fy_parameters,friction_scaling_factor);

%% run
F_z_L = 100;
F_z_R = 100;

camber = -1;
gamma_L = -camber;
gamma_R = camber;

clc
[F_y_tot, F_y_L, F_y_R, M_x_L, M_x_R, alpha_val] = ...
    singleAxleCamberEvaluation(F_z_L, F_z_R, gamma_L, gamma_R, tire)

%% test

gamma_vector = 1;
alpha_vector = -20:0.5:20;

F_y_matrix = zeros(numel(gamma_vector), numel(alpha_vector));

for i = 1:numel(gamma_vector)
    tire.gamma = gamma_vector(i);
    for j = i:numel(alpha_vector)
        F_y_matrix(i,j) = F_y(tire,alpha_vector(j),0,100);
    end
end

hold off;
for i = 1:numel(gamma_vector)
    plot(alpha_vector,F_y_matrix(i,:),'DisplayName', ['gamma = ' char(string(gamma_vector(i)))]);
    hold on;
end
legend();
xline(0);
yline(0);





