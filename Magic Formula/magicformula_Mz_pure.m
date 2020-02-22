clear all;close all;clc
%% Measured Data

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,100,150,250,350				

P_input = [12];
IA_input = [0];
FZ_input = [50 150 250];

data_file_to_fit = 'A1965run15.mat';

[alpha, Fy, Fz, Mz, ~, gamma, pi, testrange] = TireParser_Cornering(P_input, IA_input, FZ_input, data_file_to_fit);

%{
alpha0 = [linspace(14,20,100).';linspace(-20,-14,100).'];
Mz0 = zeros(200,1);
Fz0 = linspace(-250,250,200).';
gamma0 = zeros(200,1);
pi0 = 12*ones(200,1);
testrange0 = ones(200,1);

alpha = [alpha; alpha0];
Mz = [Mz; Mz0];
Fz = [Fz; Fz0];
gamma = [gamma; gamma0];
pi = [pi; pi0];
testrange = [testrange; testrange0];
%}

%% Parameters/Starting Population

a = -1;            %initial interval 
b = 1;
N = 29;          %size of chromosome = number of parameters (genes)
NP = 200;        %size of population = number of chromosomes
F = 0.4;         %disturbing factor
CP = 0.6;        %crossover probability
MP = 0.2;        %mutation probability
range = 1;       %mutation range
itermax = 2500;  %number of iterations
errmin = 1e-4;   %min percent error change

errorterminate = 0; % 1 for error termination
liveplotting = 1;   % 1 for live plotting

X = a + (b-a).*rand(NP,N); %population

errorplot = zeros(itermax,1);

tic

for iterations = 1:itermax 
%% Lateral Force Parameters

load('Fy_pure_parameters_run24_final.mat');
Xbestcell_Fy = Xbestcell;
Mz_Fycell = Mz_lateralforce_pure(Xbestcell_Fy,alpha,Fz,pi,gamma);

%% Xbest Evaluation

MzXeval = zeros(NP,numel(testrange));
for j = 1:NP
    Xeval = num2cell(X(j,:));
    MzXeval(j,:) = selfaligningmoment_pure(Xeval,alpha,Fz,pi,gamma,Mz_Fycell);
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
MzXi = selfaligningmoment_pure(Xicell,alpha,Fz,pi,gamma,Mz_Fycell);

Xnicell = num2cell(Xni);
MzXni = selfaligningmoment_pure(Xnicell,alpha,Fz,pi,gamma,Mz_Fycell);

errorXi = sum((MzXi - transpose(Mz)).^2); 
errorXni = sum((MzXni - transpose(Mz)).^2); 

rmse_Xi = sqrt(errorXi / numel(MzXi));
rmse_Xni = sqrt(errorXni / numel(MzXi));

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
    scatter(alpha(1:end),Mz(1:end));
    hold on

    Mzplot = selfaligningmoment_pure(Xbestcell,alpha,Fz,pi,gamma,Mz_Fycell);

    plot(alpha,Mzplot,'k','Linewidth',3);
    xlabel('Slip Angle','FontSize',15');
    ylabel('Self-Aligning Moment (ft-lb)','FontSize',15');
    title('FZ = 50,150,250, P = 12, IA = 0','FontSize',18)
    %grid on
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
IA_input2 = 0;
FZ_input2 = [50 150 250];

%plotting non-tested parameters
plot3 = 1;    %turn on plotting

alpha3 = linspace(-10,10,1000).';
P_input3 = [12];
IA_input3 = [0];
FZ_input3 = [250];

plot2 = 0; %turn on error plot

[alpha2, Fy2, Fz2, Mz2, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2, data_file_to_fit);
Mz_Fycell2 = Mz_lateralforce_pure(Xbestcell_Fy,alpha2,Fz2,pi2,gamma2);

figure(1);
set(gcf,'Position',[70,194,560,420]);
scatter(alpha2(1:end),Mz2(1:end));
hold on

Mzplot2 = selfaligningmoment_pure(Xbestcell,alpha2,Fz2,pi2,gamma2,Mz_Fycell2);
plot(alpha2,Mzplot2,'k','Linewidth',3);
xlabel('Slip Angle','FontSize',15');
ylabel('Mz:Self-Aligning Moment','FontSize',15');
grid on
hold on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(alpha3),1);
    pi3 = repmat(P_input3,numel(alpha3),1);
    gamma3 = repmat(IA_input3,numel(alpha3),1);

    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                Mz_Fycell3 = Mz_lateralforce_pure(Xbestcell_Fy,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b));
            end
        end
    end
        
    hold on
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                Mzplot3 = selfaligningmoment_pure(Xbestcell,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b),Mz_Fycell3);
                plot(alpha3,Mzplot3,'k','Linewidth',3);
                hold on
            end
        end
    end
    xlabel('Slip Angle','FontSize',15');
    ylabel('Self-Aligning Moment (ft-lb)','FontSize',15');
    %grid on
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

%save('Mz_pure_parameters.mat','Xbestcell','Xbestcell_Fy');
