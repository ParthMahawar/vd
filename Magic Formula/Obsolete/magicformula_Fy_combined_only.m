clear all;close all;clc
%% Measured Data

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,150,200,250
%slip angle values: 0,-3,-6

P_input = [10 12 14];
IA_input = [0 2 4];
FZ_input = [50 150 200 250];
SA_input = [0 -3 -6];

[kappa, alpha, ~, Fy, Fz, ~, gamma, pi, testrange] = TireParser_DriveBrake(P_input, IA_input, FZ_input,SA_input);

%load('Fy_pure_parameters_run24_8.mat');
%tony = Xbestcell;

%{

[alpha1, Fy1, Fz1, ~, ~, gamma1, pi1, testrange1] = TireParser_Cornering(P_input, IA_input, FZ_input);

load('A1654run24.mat','SR');
kappa1 = SR(testrange1);
kappa = [kappa; kappa1];
alpha = [alpha; alpha1];
Fy = [Fy; Fy1];
Fz = [Fz; Fz1];
gamma = [gamma; gamma1];
pi = [pi; pi1];
testrange = [testrange; testrange1];

kappa0 = linspace(-15,15,400).';
alpha0 = zeros(400,1);
Fy0 = zeros(400,1);
Fz0 = linspace(-250,250,400).';
gamma0 = zeros(400,1);
pi0 = linspace(10,14,400).';
testrange0 = ones(400,1);

kappa = [kappa; kappa0];
alpha = [alpha; alpha0];
Fy = [Fy; Fy0];
Fz = [Fz; Fz0];
gamma = [gamma; gamma0];
pi = [pi; pi0];
testrange = [testrange; testrange0];
%}


%% Parameters/Starting Population

a = 0;           %initial interval 
b = 1;
N = 15;          %number of parameters
NP = 100;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.3;        %mutation probability
range = 1;       %mutation range
itermax = 15000;    %number of iterations
errmin = 1e-4;   %min percent error change

errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N);    %initial population
errorplot = zeros(itermax,1);

%load('Fy_combined_parameters_run38_8.mat');
%X(1,:) = cell2mat(Xbestcell);

tic

for iterations = 1:itermax 
%% Xbest Evaluation

FyXeval = zeros(NP,numel(testrange));
for j = 1:NP
    Xeval = num2cell(X(j,:));
    FyXeval(j,:) = lateralforce_combined_only(Xeval,alpha,Fz,pi,gamma,kappa);
end
errorXeval = sum((FyXeval - repmat(transpose(Fy),NP,1)).^2,2); 
[Xbesterror,index] = min(errorXeval);
Xbest = X(index,:);

%Xbest(1:27) = cell2mat(tony);

Xbestcell = num2cell(Xbest);

errorplot(iterations) = Xbesterror; %for plotting error

%% Selection

for i0 = 1:NP

i1 = randi(NP);
i2 = randi(NP);
Xi = X(i0,:);
Xr1 = X(i1,:);
Xr2 = X(i2,:);
V = Xbest + F*(Xr1 - Xr2);

%% Reproduction and Mutation

crossover = rand(1);
Xni = Xi;
if crossover < CP  
    Xni = V;
    Xiselect = randi(N,[round(N/2) 1]);
    Xni(Xiselect) = Xi(Xiselect);
end

mutation = -1 + (2).*rand(1);

if abs(mutation) < MP
    Xni = Xni + sign(mutation)*range*rand(1);
end
%% Error Evaluation

Xicell = num2cell(Xi);
FyXi = lateralforce_combined_only(Xicell,alpha,Fz,pi,gamma,kappa);

Xnicell = num2cell(Xni);
FyXni = lateralforce_combined_only(Xnicell,alpha,Fz,pi,gamma,kappa);

errorXi = sum((FyXi - transpose(Fy)).^2); 
errorXni = sum((FyXni - transpose(Fy)).^2); 

if errorXni < errorXi
    Xi = Xni;
end

X(i0,:) = Xi;

end
%% Plotting

time = (toc);

clc
fprintf('iteration number: %d \nelapsed time:%.1f seconds', iterations,time);

%terminate for loop if change in percent error is below errmin
finished = 0;
if (errorterminate == 1 && iterations>200 && (mean(errorplot(iterations-200:iterations)) - Xbesterror)...
        /mean(errorplot(iterations-200:iterations-1)) < errmin) || iterations == itermax
    break
end

if ((iterations == 1 || mod(iterations,5) == 0) && liveplotting == 1)
    if iterations == 1 
        f1 = figure(1);
        set(gcf,'Position',[70,194,560,420]);
    else
        set(0,'CurrentFigure',f1);
    end
    scatter(kappa,Fy);
    hold on

    Fyplot = lateralforce_combined_only(Xbestcell,alpha,Fz,pi,gamma,kappa);

    plot(kappa,Fyplot,'k','Linewidth',3);
    xlabel('Slip Ratio','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
    grid on
    hold off
    
    if iterations == 1 
        f2 = figure(2);
        set(gcf,'Position',[656,194,560,420]);
    else
        set(0,'CurrentFigure',f2);
    end
  
    plot(1:itermax,errorplot);
    xlabel('Iterations','FontSize',15);
    ylabel('Sum-Squared Error','FontSize',15);
    if iterations<200
        itermin = 0;
    else
        itermin = iterations-200;
    end 
    xlim([itermin iterations]);
    pause(0.00001);
end

end
%% Plotting

% plotting tested parameters
P_input2 = [12];
IA_input2 = [0];
FZ_input2 = [50 150 250];
SA_input2 = [0 -3 -6];

%plotting non-tested parameters (function of kappa)
plot3 = 1;    %turn on plotting

kappa3 = linspace(-1,1,1000).';
P_input3 = [12];
IA_input3 = [0]; 
FZ_input3 = 250;
SA_input3 = [0 2 5 10 20]; %slip angle input for combined slip 

plot2 = 0; %turn on error plot

%plotting as function of alpha
plot4 = 1;  %turn on plotting

alpha4 = linspace(-30,30,1000).';
P_input4 = [12];
IA_input4 = [0];
FZ_input4 = [250];
SR_input4 = [0 0.05 0.1 0.2 0.4 0.8]; %slip ratio input for combined slip 

[kappa2, alpha2, ~, Fy2, Fz2, ~, gamma2, pi2, testrange2] = TireParser_DriveBrake(P_input2, IA_input2, FZ_input2,SA_input2);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(kappa2,Fy2);
hold on

Fyplot2 = lateralforce_combined_only(Xbestcell,alpha2,Fz2,pi2,gamma2,kappa2);
plot(kappa2,Fyplot2,'k','Linewidth',3);
xlabel('Slip Ratio','FontSize',15);
ylabel('Fy:Lateral Force','FontSize',15);
grid on
hold on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(kappa3),1);
    pi3 = repmat(P_input3,numel(kappa3),1);
    gamma3 = repmat(IA_input3,numel(kappa3),1);
    alpha3 = repmat(SA_input3,numel(kappa3),1);
    
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                for d = 1:numel(SA_input3) 
                    Fyplot3 = lateralforce_combined_only(Xbestcell,alpha3(:,d),Fz3(:,c),pi3(:,a),gamma3(:,b),...
                        kappa3);
                    plot(kappa3,Fyplot3,'k','Linewidth',3);
                    hold on
                end
            end
        end
    end
    xlabel('Slip Ratio','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
    grid on
end

if plot4 == 1
    Fz4 = -repmat(FZ_input4,numel(alpha4),1);
    pi4 = repmat(P_input4,numel(alpha4),1);
    gamma4 = repmat(IA_input4,numel(alpha4),1);
    kappa4 = repmat(SR_input4,numel(alpha4),1);
    
    figure(4)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input4)
        for b = 1:numel(IA_input4)
            for c = 1:numel(FZ_input4)
                for d = 1:numel(SR_input4) 
                    Fyplot4 = lateralforce_combined_only(Xbestcell,alpha4,Fz4(:,c),pi4(:,a),gamma4(:,b),...
                        kappa4(:,d));
                    plot(alpha4,Fyplot4,'k','Linewidth',3);
                    hold on
                end
            end
        end
    end
    xlabel('Slip Angle','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
    grid on
    
    [alpha4, Fy4, Fz4, ~, ~, gamma4, pi4, testrange4] = TireParser_Cornering(P_input4, IA_input4, FZ_input4);
    hold on
    scatter(alpha4,Fy4);
end

hold off
if plot2 == 1
    figure(2);
    set(gcf,'Position',[656,194,560,420]);
    plot(1:itermax,errorplot);
    xlabel('Iterations');
    ylabel('Sum-Squared Error');
end

%% Save Parameters

%save('Fy_combined_parameters.mat','Xbestcell');
