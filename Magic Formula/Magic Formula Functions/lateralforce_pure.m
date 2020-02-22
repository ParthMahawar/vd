function [F_y, K_y, K_ygamma] = lateralforce_pure(Xcell,alpha,F_z,p_i,gamma)
%% Inputs

F_z0 = 200*ones(size(F_z)); %nominal load
p_i0 = 13*ones(size(p_i)); %nominal pressure

alpha = alpha*0.0174533; %degrees to radians
gamma = gamma*0.0174533;  %degrees to radians
F_z = abs(F_z);

df_z = (F_z- F_z0)./F_z;
dp_i = (p_i-p_i0)./p_i;

%% Parameters

[p_cy1,p_dy1,p_dy2,p_dy3,p_ey1,p_ey2,p_ey3,p_ey4,p_ey5,p_hy1,p_hy2,p_ky1,p_ky2,p_ky3,...
    p_ky4,p_ky5,p_ky6,p_ky7,p_py1,p_py2,p_py3,p_py4,p_py5,p_vy1,p_vy2,p_vy3,p_vy4] = Xcell{:};

%p_ey3 = 0
%p_dy1 = 1.8;
%p_dy2 = -0.05;

lambda_cy = 1;       % shape factor   
lambda_ey = 1;       % curvature 
lambda_hy = 0;       % horizontal shift
lambda_kyalpha = 1;  % cornering stiffness
lambda_kygamma = 1;  % camber force stiffness
lambda_muy = 1;      % peak friction coefficient
lambda_vy = 0;       % vertical shift

%%  Magic Formula Equations

K_yalpha = p_ky1.*F_z0.*(1+p_py1.*dp_i).*sin(p_ky4.*atan(F_z./...
    ((p_ky2 + p_ky5*gamma.^2).*(1+p_py2*dp_i).*F_z0))).*...
    (1-p_ky3.*abs(gamma)).*lambda_kyalpha;
K_ygamma = (p_ky6+p_ky7.*df_z).*(1+p_py5.*dp_i).*F_z.*lambda_kygamma;
S_vy0 = F_z.*(p_vy1+p_vy2.*df_z).*lambda_vy.*lambda_muy;
S_vygamma = F_z.*(p_vy3+p_vy4.*df_z).*gamma.*lambda_kygamma.*lambda_muy;
S_vy = S_vy0+S_vygamma;
S_hy0 = (p_hy1+p_hy2.*df_z).*lambda_hy;
S_hygamma = (K_ygamma.*gamma-S_vygamma)./K_yalpha;
S_hy = S_hy0+S_hygamma;
alpha_y = alpha+S_hy;
mu_y = (p_dy1+p_dy2.*df_z).*(1-p_dy3.*gamma.^2).*...
    (1+p_py3.*dp_i+p_py4.*dp_i.^2).*lambda_muy;
C_y = p_cy1.*lambda_cy;
D_y = mu_y.*F_z;
E_y = (p_ey1+p_ey2.*df_z).*(1+p_ey5.*gamma.^2-(p_ey3+p_ey4.*gamma).*sign(alpha_y)).*lambda_ey;
B_y = K_yalpha./C_y./D_y;

%Lateral Force
F_y = transpose(D_y.*sin(C_y.*atan(B_y.*alpha_y-E_y.*(B_y.*alpha_y-atan(B_y.*alpha_y))))+S_vy);

% F_y(F_z==0) = 0; %zero load

% Cornering Stiffness
K_y = K_yalpha*-0.0174533; % rad to deg
K_ygamma = K_ygamma*-0.0174533; 

%F_y = -D_y;

%F_y = mu_y;

end