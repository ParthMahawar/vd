clear all;close all;clc

load('Fy_pure_parameters_run24_18.mat');
Xbestcell_Fy = Xbestcell;
%% Measured Data

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,100,150,250,350						

P_input = 12;
IA_input = [0];
FZ_input = [50 100 250];

data_file_to_fit = 'A1654run24.mat';

[alpha, Fy, Fz, ~, Mx, gamma, pi, testrange] = TireParser_Cornering(P_input, IA_input, FZ_input, data_file_to_fit);
%% Parameters/Starting Population

a = 0;           %initial interval 
b = 1;
N = 15;          %size of chromosome = number of parameters (genes)
NP = 100;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.1;        %mutation probability
range = 1;       %mutation range
itermax = 1500;  %number of iterations
errmin = 1e-4;   %min percent error change
errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N); %population

errorplot = zeros(itermax,1);

tic

for iterations = 1:itermax 
%% Xbest Evaluation

MxXeval = zeros(NP,numel(testrange));
for j = 1:NP
    Xeval = num2cell(X(j,:));
    MxXeval(j,:) = overturningmoment(Xeval,Fy,Fz,pi,gamma);
end
errorXeval = sum((MxXeval - repmat(transpose(Mx),NP,1)).^2,2); 
[Xbesterror,index] = min(errorXeval);
Xbest = X(index,:);
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
MxXi = overturningmoment(Xicell,Fy,Fz,pi,gamma);

Xnicell = num2cell(Xni);
MxXni = overturningmoment(Xnicell,Fy,Fz,pi,gamma);

errorXi = sum((MxXi - transpose(Mx)).^2); 
errorXni = sum((MxXni - transpose(Mx)).^2); 

rmse_Xi = sqrt(errorXi / numel(FyXi));
rmse_Xni = sqrt(errorXni / numel(FyXni));

if rmse_Xni < rmse_Xi
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
    scatter(alpha(1:5:end),Mx(1:5:end));
    hold on

    Mxplot = overturningmoment(Xbestcell,Fy,Fz,pi,gamma);

    plot(alpha,Mxplot,'k','Linewidth',3);
    xlabel('Slip Angle','FontSize',15');
    ylabel('Mx:Overturning Moment','FontSize',15');
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
IA_input2 = [0 2 4];
FZ_input2 = [50 150 350];

%plotting non-tested parameters
plot3 = 1;    %turn on plotting

alpha3 = linspace(-15,15,1000).';
P_input3 = [12];
IA_input3 = [0 2 4];
FZ_input3 = [250];
        
plot2 = 0; %turn on error plot

%tire load sensitivity (function of Fz)
plot4 = 1; %turn on plotting
alpha_input4 = linspace(-15,15,10);
P_input4 = [12];
IA_input4 = [0];
FZ4 = linspace(0,500,1000).';

[alpha2, Fy2, Fz2, ~, Mx2, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2, data_file_to_fit);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(alpha2(1:5:end),Mx2(1:5:end));
hold on

Mxplot2 = overturningmoment(Xbestcell,Fy2,Fz2,pi2,gamma2);
plot(alpha2,Mxplot2,'k','Linewidth',3);
xlabel('Slip Angle','FontSize',15');
ylabel('Mx:Overturning Moment','FontSize',15');
grid on
hold on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(alpha3),1);
    pi3 = repmat(P_input3,numel(alpha3),1);
    gamma3 = repmat(IA_input3,numel(alpha3),1);
    
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                Fy3 = lateralforce_pure(Xbestcell_Fy,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b)).';
                Mzplot3 = overturningmoment(Xbestcell,Fy3,Fz3(:,c),pi3(:,a),gamma3(:,b));
                plot(alpha3,Mzplot3,'k','Linewidth',3);
                hold on
            end
        end
    end
    xlabel('Slip Angle','FontSize',15');
    ylabel('Mx:Overturning Moment','FontSize',15');
    grid on
end

if plot4 == 1
    alpha4 = -repmat(alpha_input4,numel(FZ4),1);
    pi4 = repmat(P_input4,numel(FZ4),1);
    gamma4 = repmat(IA_input4,numel(FZ4),1);
    
    figure(4)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input4)
        for b = 1:numel(IA_input4)
            for c = 1:numel(alpha_input4)
                Fy4 = lateralforce_pure(Xbestcell_Fy,alpha4(:,c),FZ4,pi4(:,a),gamma4(:,b));
                Mzplot4 = overturningmoment(Xbestcell,Fy4,FZ4(:,c),pi4(:,a),gamma4(:,b));
                plot(FZ4,Mzplot4,'Linewidth',3);
            end
        end
    end
    xlabel('Normal Load','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
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

%save('Mx_parameters.mat','Xbestcell');
