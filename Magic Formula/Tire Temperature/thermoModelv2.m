function dTdt = thermoModel(T,Vx,Vy,Fx,Fy,Fz,V,P)
% T: vector of temperatures (C)
% [road, tread, carcass, inflation gas, ambient]

P = P*0.0689; % psi to bar

%% Input parameters (varying)

% tire = 1 - 18 R25B: 10.48 lb
%        2 - 16 R25B: 8.75 lb
%        3 - 16 LC0: 7.38 lb
tire = 3;

m_vec = [10.48 8.75 7.38]*0.454;
radius_vec = [18 16 16]*0.0254;

m = m_vec(tire); % mass of tire (kg) 
tread_proportion = 0.05; % proportion of tire mass in tread

tire_r = radius_vec(tire); % radius of tire (m)
width = 8*0.0254; % width of tire (rim) (m)

rim_r = 10*0.0254; % radius of rim (m)

%% Input parameters (constant)

% efficiencies
eta_x = 0.03;
eta_y = 0.03;
eta_z = 0.03;

R_ct = 0.5; % proportion of deflection power going to carcass instead of tread
R_rt = 1.8; % proportion of heat energy tire receives from frictional power

% conduction with ground-tread
k_21 = 1.5e3; % conduction coefficient (W/m^2-K)

w = width; % width of contact patch (m)
a = 0.5*0.11*P^(-0.7)*(abs(Fz)/3000)^0.8;
A_cp = 2*a*w; % area of contact patch (m^2)

% convection coefficient * area (W/K)
h_23 = 140; %%% unknown
h_25 = 2*V;
h_34 = 8;
h_35 = 10;

% transient properties
cp_t = 1.8e3; % specific heat capacity (tread) (J/kg-K)
cp_c = 1.8e3; % specific heat capacity (carcass) (J/kg-K)
cp_i = 1.042e3; % specific heat capacity (inflation gas) (J/kg-K)

% tire calculations
volume = pi*(tire_r^2-rim_r^2)*width; % approximate air volume
air_density = (P*100000+101325)/(287.058*(T(4)+273));

m_t = m*tread_proportion; % mass of tread (kg)
m_c = m*(1-tread_proportion); % mass of carcass (kg)
m_i = volume*air_density; % mass of inflation gas (kg) 

%% Tire Size Scaling

area = (tire_r+2*pi*(tire_r^2-rim_r^2));
nominal_area = (16*0.0254+2*pi*((16*0.0254)^2-rim_r^2));
ratio = area/nominal_area;

A_cp = A_cp*ratio; 
k_21 = k_21*ratio;
h_23 = h_23*ratio; 
h_25 = h_25*ratio;
h_34 = h_34*ratio;
h_35 = h_35*ratio;

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
    (Qdpc+Q_23-Q_34-Q_35)/(cp_c*m_c);...
    Q_34/(cp_i*m_i); 0];

