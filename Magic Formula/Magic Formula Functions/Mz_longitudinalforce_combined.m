function Mz_Fxcell = Mz_longitudinalforce_combined(Xbestcell, kappa, F_z, p_i, gamma, alpha)
%%  Inputs

gamma = gamma*0.0174533; %degrees to radians
alpha_f = alpha*0.0174533; %degrees to radians

F_z0 = round(F_z/25)*25; %nominal load
p_i0 = round(p_i); %nominal presure

df_z = (F_z- F_z0)./F_z;
dp_i = (p_i-p_i0)./p_i;

%% Parameters

[p_cx1,p_dx1,p_dx2,p_dx3,p_ex1,p_ex2,p_ex3,p_ex4,p_hx1,p_hx2,p_kx1,p_kx2,p_kx3,p_px1,...
    p_px2,p_px3,p_px4,p_vx1,p_vx2,r_bx1,r_bx2,r_bx4,r_cx1,r_ex1,r_ex2,r_hx1] = Xbestcell{:};

lambda_cx  = 1;     %shape factor
lambda_ex  = 1;     %curve factor
lambda_hx  = 1;     %horizontal shift
lambda_kxkappa = 1; %brake slip stiffness
lambda_mux = 1;     %peak friction coefficient
lambda_vx  = 1;     %vertical shift

lambda_xalpha = 1; %influence on F_x(kappa)

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

%Combined slip:
B_xalpha = (r_bx1+r_bx4.*gamma.^2).*cos(atan(r_bx2.*kappa)).*lambda_xalpha;
C_xalpha = r_cx1;
E_xalpha = r_ex1 + r_ex2.*df_z;
S_hxalpha = r_hx1;
alpha_s = alpha_f + S_hxalpha;
G_xalpha = cos(C_xalpha.*atan(B_xalpha.*alpha_s - E_xalpha.*(B_xalpha.*alpha_s - ...
    atan(B_xalpha.*alpha_s))))./cos(C_xalpha.*atan(B_xalpha.*S_hxalpha - E_xalpha...
    .*(B_xalpha.*S_hxalpha - atan(B_xalpha.*S_hxalpha))));

%Longitudinal Force:
F_x = transpose((D_x.*sin(C_x.*atan(B_x.*kappa_x-E_x.*(B_x.*kappa_x-atan(B_x.*kappa_x))))+S_vx).*G_xalpha);
Mz_Fxcell = {K_xkappa,F_x};

end

