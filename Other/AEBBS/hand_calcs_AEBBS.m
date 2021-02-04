clc
clear all;

l_w = 1.5494;
a = 0.54*l_w;
z_cg = 0.3048;
m = 179.2;
k = 1.47;

g = 9.8;

format compact;
a_l = g*k;
F_nr = m*(a*g-a_l*z_cg)/l_w
F_nf = m*g - F_nr

F_tf = F_nf/k;
F_tr = F_nr/k;

[F_tf + F_tr - m*a_l;...
F_nf + F_nr - m*g;...
F_nf*a - F_nr*(l_w - a) - (F_tr + F_tf) * z_cg]

BB = F_tf/(F_tf+F_tr)




