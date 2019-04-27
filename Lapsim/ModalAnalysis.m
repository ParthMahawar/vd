%% Car Parameters
clear; %close all; clc

load('dampingcurves.mat');
% input car parameters
car = testCar();
car.k = 200*4.45*39.37; % N/m
car.k_tf = car.k*4; % tire stiffness (N/m)
car.k_tr = car.k_tf;

% LSC-HSC, LSR_HSR
% LS ticks from fully clockwise
% HS turns from fully couter clockwise
% C12/R12 valving
% 1:5 HS sweep (0-x 0-x), x = 4.3 3 2 1 0
% 6:11 LS sweep (x-4.3 x-4.3), x = 0 2 4 6 10 15 25
car.c_compression = cdamp{7}([1 2 10:10:end],:); % damping curves ([in/s lbf])
car.c_rebound = rdamp{7}([1 2 10:10:end],:);

car.Ixx = 60;
car.Iyy = 82;
car.k_rf = 10000*6.474; % Nm/rad
car.k_rr = 10000*6.474; % Nm/rad
car.MR_F = MR18Front;
car.MR_R = MR18Front;

%% Modal Analysis (Undamped)
tirePos = zeros(1,4);
tireVel = zeros(1,4);
[m,c,k] = calc_MCK(car,tirePos,tireVel);
[V,D] = eig(inv(m)*k);

% sort eigenvalues/vectors
[d,ind] = sort(diag(D));
Ds = D(ind,ind);
Vs = V(:,ind);

% natural frequencies
omega = diag(sqrt(Ds))/(2*pi);
%% Damping Analysis
% simplified 3DOF model used (bounce,roll,pitch) for approximating damping
% ratio
% (7 DOF model has non-proportional damping so it can't be decoupled)
tirePos = zeros(1,4);
tireVel = zeros(1,4);
[m,c,k] = calc_MCK(car,tirePos,tireVel);
m = m(1:3,1:3);
c = c(1:3,1:3);
k = k(1:3,1:3);
[V,D] = eig(inv(m)*k);

% One technique for calculating damping ratios
% "A SIMPLIFIED APPROACH TO CALCULATE THE
% DAMPING RATIO MATRIX FOR MULTIPLE DEGREE OF
% FREEDOM VIBRATION SYSTEMS"
c_cr = 2*m^(0.5)*(m^(-0.5)*k*m^(-0.5))^(0.5)*m^(0.5);
eig(c-c_cr);
xi = c_cr^(-0.5)*c*c_cr^(-0.5);
xi = eig(xi);

V_norm = V*diag(sqrt(diag(V'*(m*V)).^(-1)));
c_bar = V_norm'*c*V_norm;
k_bar = V_norm'*k*V_norm;
xi2 = 0.5*diag(c_bar)./diag(sqrt(k_bar));

xi2 = [xi2; zeros(4,1)];

figure
for n = 1:7
    subplot(7,1,n)
    [maximum,index] = max(abs(Vs(:,n)));
    plot(Vs(:,n)/maximum*sign(Vs(index,n)),'--o');
    title(['\omega =' num2str(omega(n)) ' xi =' num2str(xi2(n))])
    set(gcf,'Position',[184 66 600 714]);
    set(gca,'XTick',1:7,'XTickLabel',{'Bounce','Roll','Pitch',...
        'FL Tire' 'FR Tire' 'RL Tire' 'RR Tire'})
end

%%
[X,e,s] = polyeig(k,c,m);
%xi = real(e)./imag(e);
xi3 = (ones(size(e))./((imag(e)./real(e)).^2+1)).^0.5;

omega2 = abs(e);
omega2 = omega2(1:2:end);
[~,omega2sort] = sort(omega2);
xi4 = 0.5*diag(c_bar)./omega2;
xi4 = xi4(omega2sort);

%%
close all

[X,e,s] = polyeig(k,c,m);
xi4 = (ones(size(e))./((imag(e)./real(e)).^2+1)).^0.5;
omega4 = abs(e)/(2*pi);

xi4_undamped = xi4(xi4<1)

[maxX, idxX] = max(abs(X));
for j=1:size(X,2)
   %X(:,j) = X(:,j)*exp(-1i*angle(X(idxX(j),j))); % rotate entire e-vector
   X(:,j) = X(:,j)/maxX(j); % normalize
end

for n = 1:7
    subplot(7,1,n)
    sgn = sign(real(X(:,n)));
    sgn(sgn==0) = 1;   
    mag = abs(X(:,n)).*sgn;
    [maxM,idxM] = max(abs(mag));
    plot(1:7,mag.*sign(mag(idxM)),'--o');
    title(['\omega =' num2str(omega4(n)) ' xi =' num2str(xi4(n))])
    set(gcf,'Position',[184 66 600 714]);
    set(gca,'XTick',1:7,'XTickLabel',{'Bounce','Roll','Pitch',...
        'FL Tire' 'FR Tire' 'RL Tire' 'RR Tire'})
end

figure
for n = 1:7
    subplot(7,1,n)
    sgn = sign(real(X(:,n+7)));
    sgn(sgn==0) = 1;   
    mag = abs(X(:,n+7)).*sgn;
    [maxM,idxM] = max(abs(mag));
    plot(1:7,mag.*sign(mag(idxM)),'--o');
    title(['\omega =' num2str(omega4(n+7)) ' xi =' num2str(xi4(n+7))])
    set(gcf,'Position',[184 66 600 714]);
    set(gca,'XTick',1:7,'XTickLabel',{'Bounce','Roll','Pitch',...
        'FL Tire' 'FR Tire' 'RL Tire' 'RR Tire'})
end

%%


%% Quarter Car Model
MR = interp1(car.MR_F(:,1),car.MR_F(:,2),0);
wheel_rate = car.k*MR^2;
omega_n = sqrt(wheel_rate/(car.M/4))/(2*pi) %Hz
c_cr = 2*sqrt(wheel_rate*(car.M/4)); %Ns/m
c = (car.c_compression(2,2)-car.c_compression(1,2))/(car.c_compression(2,1)-car.c_compression(1,1))*175.126835;
xi = c/c_cr

%% Verification
m = [1 0;0 2];
m = [1 -1; -1 20];
m = [1 0; 0 2];
c = [9 -5.656;-5.656 8];
c = [0.1 -0.1;-0.1 0.7];
c = [6 -1.414; -1.414 1];
k = [3 -1.414; -1.414 2];
k = [20 -20; -20 50];
k = [10 -1.414; -1.414 1];
    
[V,D] = eig(inv(m)*k);

% mass normalized eigenvectors
V_norm = V*diag(sqrt(diag(V'*(m*V)).^(-1)));
c_bar = V_norm'*c*V_norm;
k_bar = V_norm'*k*V_norm;

% sort eigenvalues/vectors
[d,ind] = sort(diag(D));
Ds = D(ind,ind);
Vs = V(:,ind);

% natural[ frequencies
omega = diag(sqrt(Ds))/(2*pi);
omega = omega*2*pi;

c_cr = 2*m^(0.5)*(m^(-0.5)*k*m^(-0.5))^(0.5)*m^(0.5);
eig(c-c_cr);
xi = c_cr^(-0.5)*c*c_cr^(-0.5);
eig(xi)

0.5*diag(c_bar)./diag(sqrt(k_bar))

%%
[X,e,s] = polyeig(k,c,m);
xi = real(e)./imag(e);
xi3 = (ones(size(e))./((imag(e)./real(e)).^2+1)).^0.5;

omega2 = abs(e);
omega2 = omega2(1:2:end);
[~,omega2sort] = sort(omega2);
xi = 0.5*diag(c_bar)./omega2;
xi = xi(omega2sort);

%X_norm = X*diag(sqrt(diag(X'*(m*X)).^(-1)));


%%
A = [zeros(size(m)) eye(size(m));		
      -m\k     -m\c  ];
  
[R,Lambda] = eig(A);

Lambda = diag(Lambda);
[wn,idx] = sort(abs(Lambda));	% sort natural frequencies in increasing order
Lambda = Lambda(idx);           % sort eigenvalues  in increasing order
R = R(:,idx);			% sort eigenvectors in increasing order
  
% rotate each eigenvector [ r ; lambda*r ] so that the largest magnitude in the
% top half (r, the displacement half) is purely real 
 [maxR, idxR] = max(abs(R(1:size(k,1),:)));
 for j=1:size(R,1)
   R(:,j) = R(:,j) * exp(-i*angle(R(idxR(j),j)));    % rotate entire e-vector
 end
%Phases = angle(R)


% Make the displacement parts of the mode-shapes "mass-normalized"
% The imaginary part of r'*Ms*r is due only to floating-point rounding error
% and is discarded by retaining only the real part of r'*Ms*r, 
% even for complex displacement mode vectors. 
 for j=1:size(R,1)
   R(:,j) = R(:,j) / sqrt(real(R(1:size(k,1),j)'*m*R(1:size(k,1),j)));
   if real(R(1,j)) < 0, R(:,j) = -R(:,j); end
 end


