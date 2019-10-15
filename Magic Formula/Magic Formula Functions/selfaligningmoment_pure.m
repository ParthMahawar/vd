function M_z = selfaligningmoment_pure(Xcell,alpha,F_z,p_i,gamma,Mz_Fycell)
%% Inputs

gamma = gamma.*0.0174533;  %degrees to radians
alpha_m = alpha*0.0174533;        

R_0 = 9.15;  %unloaded tire radius

F_z0 = 200*ones(size(F_z)); %nominal load
p_i0 = 13*ones(size(p_i)); %nominal pressure
F_z = abs(F_z);

df_z = (F_z- F_z0)./F_z; 
dp_i = (p_i-p_i0)./p_i;

%% Parameters
[p_pz1,p_pz2,q_bz1,q_bz2,q_bz3,q_bz4,q_bz5,q_bz9,q_bz10,q_cz1,q_dz1,q_dz2,q_dz3,q_dz4,q_dz6,q_dz7,q_dz8,q_dz9,q_dz10,q_dz11,...
    q_ez1,q_ez2,q_ez3,q_ez4,q_ez5,q_hz1,q_hz2,q_hz3,q_hz4] = Xcell{:};

lambda_t = 1;        % pneumatic trail (affecting aligning torque stiffness)    
lambda_r = 1;        % curvature 
lambda_kyalpha = 1;  % cornering stiffness
lambda_kzgamma = 1;  % camber force stiffness
lambda_muy = 1;      % peak friction coefficient

[S_hy,S_vy,K_yalpha,B_y,C_y,F_y] = Mz_Fycell{:}; %lateral force parameters

F_y = transpose(F_y);

%% Magic Formula Equations

S_ht = q_hz1+q_hz2.*df_z+(q_hz3+q_hz4.*df_z).*gamma;
alpha_r = alpha_m+S_hy+S_vy./K_yalpha;
alpha_t = alpha_m + S_ht;

alpha_teq = alpha_t;
alpha_req = alpha_r;

%Pneumatic trail t:
B_t = (q_bz1+q_bz2.*df_z+q_bz3.*df_z.^2).*(1+q_bz4+q_bz5.*abs(gamma)).*lambda_kyalpha./lambda_muy;
C_t = q_cz1;
D_t = (q_dz1+q_dz2.*df_z).*(1-p_pz1.*dp_i).*(1+q_dz3.*gamma+q_dz4.*gamma.^2).*F_z.*R_0./F_z0.*lambda_t;
E_t = (q_ez1+q_ez2.*df_z+q_ez3.*df_z.^2).*(1+(q_ez4+q_ez5.*gamma).*(2./3.14159).*atan(B_t.*C_t.*alpha_t));
t = D_t.*cos(C_t.*atan(B_t.*alpha_teq-E_t.*(B_t.*alpha_teq-atan(B_t.*alpha_teq)))).*cos(alpha_m);

%Residual moment M_zr:
B_r = q_bz9.*lambda_kyalpha./lambda_muy+q_bz10.*B_y.*C_y;
D_r = ((q_dz6+q_dz7.*df_z).*lambda_r+(q_dz8+q_dz9.*df_z).*(1-p_pz2.*dp_i).*gamma.*lambda_kzgamma...
    +(q_dz10+q_dz11.*df_z).*gamma.*abs(gamma).*lambda_kzgamma).*F_z.*R_0.*lambda_muy;
M_zr = D_r.*cos(atan(B_r.*alpha_req)).*cos(alpha_m);

%Self aligning moment:
M_z = transpose(-t.*F_y+M_zr);

%M_z = D_t;

%M_z = t*12;
