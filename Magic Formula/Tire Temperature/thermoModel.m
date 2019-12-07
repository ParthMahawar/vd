function dTdt = thermoModel(T,Vx,Vy,Fx,Fy,Fz,V,P)
% T: vector of temperatures (C)
% [road, tread, carcass, inflation gas, ambient]

P = P*0.0689; % psi to bar

%% Input parameters 
% efficiencies
eta_x = 0.03;
eta_y = 0.03;
eta_z = 0.03;

R_ct = 0.6; % proportion of deflection power going to carcass instead of tread
R_rt = 0.6; % proportion of heat energy tire receives from frictional power

% conduction with ground-tread
k_21 = 0.8*12e3; % conduction coefficient (W/m^2-K)

w = 0.2; % width of contact patch (m)
a = 0.5*0.11*P^(-0.7)*(abs(Fz)/3000)^0.8;
A_cp = 2*a*w; % area of contact patch (m^2)

% convection coefficient * area (W/K)
h_23 = 100; %%% unknown
h_25 = 2*V-30; 
h_34 = 8;
h_35 = 30;

% transient properties
cp_t = 1.8e3; % specific heat capacity (tread) (J/kg-K)
cp_c = 1.8e3; % specific heat capacity (carcass) (J/kg-K)
cp_i = 1.042e3; % specific heat capacity (inflation gas) (J/kg-K)

m_t = 7.38*0.454*0.13; %9.5; % mass of tread (kg)
m_c = 7.38*0.454*0.87; %0.5 % mass of carcass (kg)
m_i = 0.05; % mass of inflation gas (kg) %%% unknown

%% Heat Generation 

% deflection power
Qdpx = eta_x*V*abs(Fx); % W
Qdpy = eta_y*V*abs(Fy); % W
Qdpz = eta_z*V*abs(Fz); % W

Qdp_net = Qdpx+Qdpy+Qdpz; % net power
Qdpt = Qdp_net*R_ct; % power to tread
Qdpc = Qdp_net*(1-R_ct); % power to carcass

% frictional power
Qfp = (abs(Vx)*abs(Fx)+abs(Vy)*abs(Fy))*R_rt;

%% Heat Transfer

Q_21 = k_21*(T(2)-T(1))*A_cp;
Q_23 = h_23*(T(2)-T(3)); 
Q_25 = h_25*(T(2)-T(5));
Q_34 = h_34*(T(3)-T(4));
Q_35 = h_35*(T(3)-T(5));

% pressure scaled by T(4)

%% Rate of Temperature Change

% T: vector of temperatures 
% [road, tread, carcass, inflation gas, ambient]
dTdt = [0; (Qfp+Qdpt-Q_21-Q_23-Q_25)/(cp_t*m_t);...
    (Qdpc+Q_23+Q_34+Q_35)/(cp_c*m_c);...
    Q_34/(cp_i*m_i); 0];


