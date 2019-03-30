function M_x = overturningmoment(Xcell,F_y,F_z,p_i,gamma)
%% Inputs

gamma = gamma*0.0174533; %degrees to radians

R_0 = 9;  %unloaded tire radius

F_z0 = 200*ones(size(F_z)); %nominal load
p_i0 = 13*ones(size(p_i)); %nominal pressure
F_z = abs(F_z);

dp_i = (p_i-p_i0)./p_i;

%% Parameters

[p_pmx1,q_sx1,q_sx2,q_sx3,q_sx4,q_sx5,q_sx6,q_sx7,q_sx8,q_sx9,q_sx10,q_sx11,q_sx12,q_sx13,q_sx14] = Xcell{:};

lambda_mx = 1;  %overturning couple stiffness
lambda_vmx = 1; %overturning couple vertical shift

%%  Magic Formula Equations

M_x = transpose(R_0.*F_z.*lambda_mx.*(q_sx1.*lambda_vmx-q_sx2.*gamma.*(1+p_pmx1.*dp_i)-q_sx12).*gamma.*abs(gamma)...
    +q_sx3.*F_y./F_z0+q_sx4.*cos(q_sx5.*atan((q_sx6.*F_z.*F_z0).^2)).*sin(q_sx7.*gamma+q_sx8.*atan(q_sx9.*F_y./F_z0)...
    +q_sx10.*atan(q_sx11.*F_z./F_z0).*gamma)+R_0.*F_y.*lambda_mx.*(q_sx13+q_sx14.*abs(gamma)));

end