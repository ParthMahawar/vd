function x_dot = BicycleODE(t,x,a,b,u,delta,C_f,C_r,m,I_zz,Xcell)

%m*u*(beta_dot+r) = Fy_1+Fy_2
%I_zz*r_dot = a*Fy_1+b*Fy_2

deltaval = @(t) 10*pi/180*(t>0);
delta = deltaval(t);

r = x(1);
beta = x(2);

alpha_f = beta+a*r/u-delta;
alpha_r = beta-b*r/u;

%alpha_r*180/pi

%Fy_1 = -C_f*alpha_f;
%Fy_2 = -C_r*alpha_r;

Fy_1 = lateralforce_pure(Xcell,alpha_f,m*a/(a+b),12,0);
Fy_2 = lateralforce_pure(Xcell,alpha_r,m*b/(a+b),12,0);

r_dot = (a*Fy_1-b*Fy_2)/I_zz;
beta_dot = (Fy_1+Fy_2)/(m*u)-r;

x_dot = [r_dot;beta_dot];

end