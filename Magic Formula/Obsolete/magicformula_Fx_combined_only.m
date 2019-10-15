close all;clc
%% Measured Data

%pressures: 10,12,14
%camber schedule: 0,2,4
%FZ: 50,150,200,250	
%SA: 0,-3,-6

P_input = [10 12 14];
IA_input = [0 2 4];
FZ_input = [50 150 200 250];
SA_input = [0 -3 -6];

[kappa, alpha, Fx, ~, Fz, ~, gamma, pi, testrange] = TireParser_DriveBrake(P_input, IA_input, FZ_input,SA_input);

%{
kappa = [kappa; kappa];
alpha = [alpha; -alpha];
Fx = [Fx; Fx];
Fz = [Fz; Fz];
gamma = [gamma; gamma];
pi = [pi;pi];
testrange = [testrange;testrange];
%}

%% Parameters/Starting Population

a = 0;           %initial interval 
b = 1;
N = 7;          %number of parameters
NP = 100;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.4;        %mutation probability
range = 1;       %mutation range
itermax = 5000;  %number of iterations
errmin = 1e-4;   %min percent error change

errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N);  %population

%load('Fx_combined_parameters_run38_9.mat');

%X = (a + (b-a).*rand(NP,N))+repmat(cell2mat(Xbestcell),NP,1);    %initial population

%X(:,11) = 60*ones(NP,1) -2 + 4.*rand(NP,1);

%X(1,:) = cell2mat(Xbestcell);

errorplot = zeros(itermax,1);
tic

for iterations = 1:itermax 
%% Xbest Evaluation

FyXeval = zeros(NP,numel(testrange));
for j = 1:NP
    Xeval = num2cell(X(j,:));
    FyXeval(j,:) = longitudinalforce_combined_only(Xeval,kappa,Fz,pi,gamma,alpha);
end
errorXeval = sum((FyXeval - repmat(transpose(Fx),NP,1)).^2,2); 
[Xbesterror,index] = min(errorXeval);
Xbest = X(index,:);

Xbest(7) = 0; %for symmetry

Xbestcell = num2cell(Xbest);

errorplot(iterations) = Xbesterror; %for plotting error

%% Selection

for i0 = 1:NP;
    
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
FxXi = longitudinalforce_combined_only(Xicell,kappa,Fz,pi,gamma,alpha);

Xnicell = num2cell(Xni);
FyXni = longitudinalforce_combined_only(Xnicell,kappa,Fz,pi,gamma,alpha);

errorXi = sum((FxXi - transpose(Fx)).^2); 
errorXni = sum((FyXni - transpose(Fx)).^2); 

if errorXni < errorXi
    Xi = Xni;
end

X(i0,:) = Xi;

end
%% Live Plotting
time = (toc);

clc
fprintf('iteration number: %d \nelapsed time:%.1f seconds', iterations,time);

%terminate for loop if change in percent error is below errmin
if (errorterminate == 1 && iterations>200 && (mean(errorplot(iterations-200:iterations)) - Xbesterror)...
        /mean(errorplot(iterations-200:iterations-1)) < errmin) || iterations == itermax
    break
end

if (iterations == 1 || mod(iterations,5) == 0) && liveplotting == 1
    if iterations == 1 
        f1 = figure(1);
        set(gcf,'Position',[70,194,560,420]);
    else
        set(0,'CurrentFigure',f1);
    end
    scatter(kappa,Fx);
    hold on

    Fxplot = longitudinalforce_combined_only(Xbestcell,kappa,Fz,pi,gamma,alpha);

    plot(kappa,Fxplot,'k','Linewidth',3);
    xlabel('Slip Ratio','FontSize',15);
    ylabel('Fx:Longitudinal Force','FontSize',15);
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
IA_input2 = [4];
FZ_input2 = [50 150 200 250];
SA_input2 = [0];

%plotting non-tested parameters (function of kappa)
plot3 = 1;    %turn on plotting

kappa3 = linspace(-1,1,1000).';
P_input3 = [12];
IA_input3 = [0];
FZ_input3 = [250];
SA_input3 = [0 0.05 0.1 0.2 0.4];

plot2 = 0; %turn on error plot

%plotting as function of alpha
plot4 = 1;

alpha4 = linspace(-15,15,1000).';
P_input4 = 12;
IA_input4 = [0];
FZ_input4 = [250];
SR_input4 = [0];

[kappa2, alpha2, Fx2, ~, Fz2, ~,gamma2, pi2, testrange2] = TireParser_DriveBrake(P_input2, IA_input2, FZ_input2,SA_input2);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(kappa2,Fx2);
hold on

Fxplot2 = longitudinalforce_combined_only(Xbestcell,kappa2,Fz2,pi2,gamma2,alpha2);
plot(kappa2,Fxplot2,'k','Linewidth',3);
xlabel('Slip Ratio','FontSize',15);
ylabel('Fx:Longitudinal Force','FontSize',15);
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
                    Fxplot3 = longitudinalforce_combined_only(Xbestcell,kappa3,Fz3(:,c),pi3(:,a),gamma3(:,b),alpha3(:,d));
                    plot(kappa3,Fxplot3,'k','Linewidth',3);
                    hold on
                end
            end
        end
    end
    xlabel('Slip Ratio','FontSize',15);
    ylabel('Fx:Longitudinal Force','FontSize',15);
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
                    Fxplot4 = longitudinalforce_combined_only(Xbestcell,kappa4(:,d),Fz4(:,c),pi4(:,a),...
                        gamma4(:,b),alpha4);
                    plot(alpha4,Fxplot4,'k','Linewidth',3);
                    hold on
                end
            end
        end
    end
    xlabel('Slip Angle','FontSize',15);
    ylabel('Fx:Longitudinal Force','FontSize',15);
    grid on
end

hold off
if plot2 == 1
    figure(2);
    set(gcf,'Position',[656,194,560,420]);
    plot(1:itermax,errorplot);
    xlabel('Iterations','FontSize',15);
    ylabel('Sum-Squared Error','FontSize',15);
end

%% Save Parameters

%save('Fx_combined_parameters.mat','Xbestcell');
