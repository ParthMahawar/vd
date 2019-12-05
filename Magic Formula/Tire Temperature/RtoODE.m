function solpts = RtoODE(r,tspan,y0)

load('A1965raw6.mat')
V = 25; % mph
V = V*2.44704; % m/s

Vx = 0;
Vy = V*tand(SA);

P = 10; % psi

% T: vector of temperatures (C)
% [road, tread, carcass, inflation gas, ambient]
road = 80; % C (RST)
tread = 85; % C (TSTC)
carcass = 85; % C
gas = 85; % C
ambient = 80.5; % C

T0 = [road tread carcass gas ambient]';
T = T0;

params = r;
sol = ode45(@(t,T) thermoModelODE(T,Vx,Vy,FX*4.45,FY*4.45,FZ*4.45,V,P,params,ET,t),tspan,y0);
solpts = deval(sol,tspan);
end