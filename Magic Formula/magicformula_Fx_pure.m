clear all;close all;clc
%% Measured Data

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,150,200,250
%slip angle is 0 for pure slip

P_input = [10 12 14];
IA_input = [0 2 4];
FZ_input = [50 150 250];
SA_input = 0;

[kappa, ~, Fx, ~, Fz, ~, gamma, pi, testrange] = TireParser_DriveBrake(P_input, IA_input, FZ_input,SA_input);

%% Parameters/Starting Population

a = -1;           %initial interval
b = 1;
N = 19;          %number of parameters
NP = 100;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.4;        %mutation probability
range = 1;       %mutation range
itermax = 5000;   %number of iterations
errmin = 1e-4;   %min percent error change

errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N);  %initial population

%load('Fx_pure_parameters_run38_3.mat');

%X = (a + (b-a).*rand(NP,N))+repmat(cell2mat(Xbestcell),NP,1);    %initial population

%X(1,:) = cell2mat(Xbestcell);

%X(:,3) = zeros(NP,1);
%X(:,12) = zeros(NP,1);

%Xbest(3) = 0;  %pdx2
%Xbest(12) = 0; %pkx2
%Xbest(6) = 0; %pex2

%Xbest(7) = 0; %pex3
%Xbest(9) = 0;  %phx1
%Xbest(10) = 0; %phx2
%Xbest(18) = 0; %pvx1
%Xbest(19) = 0; %pvx2

%X(:,11) = 17*ones(NP,1) -2 + 4.*rand(NP,1);

errorplot = zeros(itermax,1);
tic

for iterations = 1:itermax
    %% Xbest Evaluation
    
    FyXeval = zeros(NP,numel(testrange));
    for j = 1:NP
        Xeval = num2cell(X(j,:));
        FyXeval(j,:) = longitudinalforce_pure(Xeval,kappa,Fz,pi,gamma);
    end
    errorXeval = sum((FyXeval - repmat(transpose(Fx),NP,1)).^2,2);
    [Xbesterror,index] = min(errorXeval);
    Xbest = X(index,:);
    
    if(Xbest(1))<1.3
        Xbest(1) = 1.3;
    end
    
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
        FxXi = longitudinalforce_pure(Xicell,kappa,Fz,pi,gamma);
        
        Xnicell = num2cell(Xni);
        FyXni = longitudinalforce_pure(Xnicell,kappa,Fz,pi,gamma);
        
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
        
        Fxplot = longitudinalforce_pure(Xbestcell,kappa,Fz,pi,gamma);
        
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
IA_input2 = [0];
FZ_input2 = [50 150 250];
SA_input2 = 0;

%plotting non-tested parameters
plot3 = 1;    %turn on plotting
kappa3 = linspace(-1,1,1000).';
P_input3 = [12];
IA_input3 = [0];
FZ_input3 = [50 150 250];

%tire load sensitivity (function of Fz)
plot4 = 1; %turn on plotting
kappa_input4 = linspace(-0.2,0.2,10);
P_input4 = [12];
IA_input4 = [0];
FZ4 = linspace(25,500,1000).';

plot2 = 0;    %turn on error plot

[kappa2, ~, Fx2, ~, Fz2, ~,gamma2, pi2, testrange2] = TireParser_DriveBrake(P_input2, IA_input2, FZ_input2,SA_input2);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(kappa2,Fx2);
hold on

Fxplot2 = longitudinalforce_pure(Xbestcell,kappa2,Fz2,pi2,gamma2);
plot(kappa2,Fxplot2,'k','Linewidth',3);
xlabel('Slip Ratio','FontSize',15);
ylabel('Fx:Longitudinal Force','FontSize',15);
grid on
hold on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(kappa3),1);
    pi3 = repmat(P_input3,numel(kappa3),1);
    gamma3 = repmat(IA_input3,numel(kappa3),1);
    
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                Fxplot3 = longitudinalforce_pure(Xbestcell,kappa3,Fz3(:,c),pi3(:,a),gamma3(:,b));
                plot(kappa3,Fxplot3,'k','Linewidth',3);
                hold on
            end
        end
    end
    xlabel('Slip Ratio','FontSize',15);
    ylabel('Fx:Longitudinal Force','FontSize',15);
    grid on
end

if plot4 == 1
    kappa4 = -repmat(kappa_input4,numel(FZ4),1);
    pi4 = repmat(P_input4,numel(FZ4),1);
    gamma4 = repmat(IA_input4,numel(FZ4),1);
    
    figure(4)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input4)
        for b = 1:numel(IA_input4)
            for c = 1:numel(kappa_input4)
                Fxplot4 = longitudinalforce_pure(Xbestcell,kappa4(:,c),FZ4,pi3(:,a),gamma4(:,b));
                plot(FZ4,Fxplot4,'Linewidth',3);
                hold on
                FZ = FZ4;
                mu = Fxplot4;
            end
        end
    end
    xlabel('Normal Load','FontSize',15);
    ylabel('Fx:Longitudinal Force','FontSize',15);
    grid on
    legend('1','2','3','4','5','6','7');
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

%save('Fx_pure_parameters.mat','Xbestcell');