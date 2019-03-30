function F_x = longitudinalforce_pure(Xcell, kappa, F_z, p_i, gamma)
%%  Inputs
F_z0 = 200*ones(size(F_z)); %nominal load
p_i0 = 13*ones(size(p_i)); %nominal pressure

gamma = gamma*0.0174533;  %degrees to radians
F_z = abs(F_z);

df_z = (F_z- F_z0)./F_z;
dp_i = (p_i-p_i0)./p_i;

%% Parameters

[p_cx1,p_dx1,p_dx2,p_dx3,p_ex1,p_ex2,p_ex3,p_ex4,p_hx1,p_hx2,p_kx1,p_kx2,p_kx3,p_px1,...
    p_px2,p_px3,p_px4,p_vx1,p_vx2] = Xcell{:};

lambda_cx  = 1;     %shape factor
lambda_ex  = 1;     %curve factor
lambda_hx  = 0;     %horizontal shift
lambda_kxkappa = 1; %brake slip stiffness
lambda_mux = 0.55;     %peak friction coefficient
lambda_vx  = 0;     %vertical shift

%% Magic Formula Equations

mu_x = (p_dx1+p_dx2.*df_z).*(1-p_dx3.*gamma.^2).*(1+p_px3.*dp_i+p_px4.*dp_i.^2).*lambda_mux;
K_xkappa = (p_kx1+p_kx2.*df_z).*exp(p_kx3.*df_z).*(1+p_px1.*dp_i+p_px2.*dp_i.^2).*F_z.*lambda_kxkappa;
S_hx = (p_hx1+p_hx2.*df_z).*lambda_hx;
S_vx = (p_vx1+p_vx2.*df_z).*F_z.*lambda_vx.*lambda_mux;
kappa_x = kappa + S_hx;
C_x = p_cx1.*lambda_cx;
D_x = mu_x.*F_z;
E_x = (p_ex1+p_ex2.*df_z+p_ex3.*df_z.^2).*(1-p_ex4.*sign(kappa_x)).*lambda_ex;
B_x = K_xkappa./(C_x.*D_x);

%Longitudinal Force
F_x = transpose(D_x.*sin(C_x.*atan(B_x.*kappa_x-E_x.*(B_x.*kappa_x-atan(B_x.*kappa_x))))+S_vx);

%F_x = mu_x;

F_x(F_z==0) = 0; %zero load
end

