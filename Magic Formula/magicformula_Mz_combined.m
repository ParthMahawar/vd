clear all;close all;clc
%% Measured Data

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values:50,150,250,350				

P_input = 12;
IA_input = 0;
FZ_input = [150];
SA_input = [0 -3 -6];

[kappa, alpha, Fx, Fy, Fz, Mz, gamma, pi, testrange] = TireParser_DriveBrake(P_input, IA_input, FZ_input,SA_input);

%% Parameters/Starting Population

a = 0;           %initial interval 
b = 1;
N = 33;          %number of parameters
NP = 100;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.1;        %mutation probability
range = 1;       %mutation range
itermax = 1000;  %number of iterations
errmin = 1e-4;   %min percent error change

errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N); %population

errorplot = zeros(itermax,1);

tic

for iterations = 1:itermax 
%% Lateral/Longitudinal Force Parameters

load('Fx_combined_parameters.mat');
Xbestcell_Fx = Xbestcell;
Mz_Fxcell = Mz_longitudinalforce_combined(Xbestcell_Fx,kappa,Fz,pi,gamma,alpha);

load('Fy_combined_parameters.mat');
Xbestcell_Fy = Xbestcell;
Mz_Fycell = Mz_lateralforce_combined(Xbestcell_Fy,alpha,Fz,pi,gamma,kappa);

%% Xbest Evaluation

MzXeval = zeros(NP,numel(testrange));
for j = 1:NP
    Xeval = num2cell(X(j,:));
    MzXeval(j,:) = selfaligningmoment_combined(Xeval,alpha,Fz,pi,gamma,Mz_Fycell,kappa,Mz_Fxcell);
end
errorXeval = sum((MzXeval - repmat(transpose(Mz),NP,1)).^2,2); 
[Xbesterror,index] = min(errorXeval);
Xbest = X(index,:);
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
MzXi = selfaligningmoment_combined(Xicell,alpha,Fz,pi,gamma,Mz_Fycell,kappa,Mz_Fxcell);

Xnicell = num2cell(Xni);
MzXni = selfaligningmoment_combined(Xnicell,alpha,Fz,pi,gamma,Mz_Fycell,kappa,Mz_Fxcell);

errorXi = sum((MzXi - transpose(Mz)).^2); 
errorXni = sum((MzXni - transpose(Mz)).^2); 

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
    scatter(kappa(1:5:end),Mz(1:5:end));
    hold on

    Mzplot = selfaligningmoment_combined(Xbestcell,alpha,Fz,pi,gamma,Mz_Fycell,kappa,Mz_Fxcell);

    plot(kappa,Mzplot,'k','Linewidth',3);
    xlabel('Slip Ratio','FontSize',15');
    ylabel('Mz:Self-Aligning Moment','FontSize',15');
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

Xbestcell = num2cell(Xbest);

%% Plotting

% plotting tested parameters
P_input2 = [12];
IA_input2 = 0;
FZ_input2 = [150];
SA_input2 = [0 -3 -6];

%plotting non-tested parameters (function of alpha)
plot3 = 1;    %turn on plotting

alpha3 = linspace(-15,15,1000).';
P_input3 = [12];
IA_input3 = [0];
FZ_input3 = 250;
SR_input3 = [0];

[kappa2, alpha2, Fx2, ~, Fz2, Mz2, gamma2, pi2, testrange2] = TireParser_DriveBrake(P_input2, IA_input2, FZ_input2,SA_input2);
Mz_Fxcell2 = Mz_longitudinalforce_combined(Xbestcell_Fx,kappa2,Fz2,pi2,gamma2,alpha2);
Mz_Fycell2 = Mz_lateralforce_combined(Xbestcell_Fy,alpha2,Fz2,pi2,gamma2,kappa2);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(kappa2(1:5:end),Mz2(1:5:end));
hold on

Mzplot2 = selfaligningmoment_combined(Xbestcell,alpha2,Fz2,pi2,gamma2,Mz_Fycell2,kappa2,Mz_Fxcell2);
plot(kappa2,Mzplot2,'k','Linewidth',3);
xlabel('Slip Ratio','FontSize',15');
ylabel('Mz:Self-Aligning Moment','FontSize',15');
grid on
hold on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(alpha3),1);
    pi3 = repmat(P_input3,numel(alpha3),1);
    gamma3 = repmat(IA_input3,numel(alpha3),1);
    kappa3 = repmat(SR_input3,numel(alpha3),1);

    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                for d = 1:numel(SR_input3)
                    Mz_Fycell3 = Mz_lateralforce_combined(Xbestcell_Fy,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b),...
                    kappa3(:,d));
                end
            end
        end
    end
    
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                for d = 1:numel(SR_input3)                 
                    Mz_Fxcell3 = Mz_longitudinalforce_combined(Xbestcell_Fx,kappa3(:,d),Fz3(:,c),pi3(:,a),...
                        gamma3(:,b),alpha3);
                end
            end
        end
    end
    end
        
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                for d = 1:numel(SR_input3)   
                    Mzplot3 = selfaligningmoment_combined(Xbestcell,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b),...
                    Mz_Fycell3,kappa3(:,d),Mz_Fxcell3);
                    plot(alpha3,Mzplot3,'k','Linewidth',3);
                    hold on
                end
            end
        end
    end
    xlabel('Slip Angle','FontSize',15');
    ylabel('Mz:Self-Aligning Moment','FontSize',15');
    grid on

hold off
figure(2);
set(gcf,'Position',[656,194,560,420]);
plot(1:itermax,errorplot);
xlabel('Iterations');
ylabel('Sum-Squared Error');

%% Save Parameters

%save('Mz_combined_parameters.mat','Xbestcell','Xbestcell_Fx','Xbestcell_Fy');
