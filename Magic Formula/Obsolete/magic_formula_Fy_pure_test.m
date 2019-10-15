%clear all;close all;clc
%% Measured Data				

%pressure values: 10,12,14
%camber values: 0,2,4
%normal force values: 50,100,150,200,250

P_input = [10 12 14];           
IA_input = [0 2 4];           
FZ_input = [50 150 250];  			

[alpha, Fy, Fz, ~, ~, gamma, pi, testrange] = TireParser_Cornering(P_input, IA_input, FZ_input);

fitnessfn = @(X) sum((lateralforce_pure_test(X,alpha,Fz,pi,gamma) - transpose(Fy)).^2); 

x = ga(fitnessfn,27);


%% Plotting
Xbestcell = x;

% plotting tested parameters
P_input2 = [12];
IA_input2 = [0];
FZ_input2 = [50 150 250];

%plotting non-tested parameters
plot3 = 1;    %turn on plotting
alpha3 = linspace(-20,20,1000).';
P_input3 = [12];
IA_input3 = [-3 0 3];
FZ_input3 = [150];

plot2 = 0;    %turn on error plot

[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, IA_input2, FZ_input2);

figure(1);
scatter(alpha2,Fy2);
hold on
set(gcf,'Position',[70,194,560,420]);
plot(alpha2,Fy2,'b');
hold on

%{
[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, 2, FZ_input2);
scatter(alpha2,Fy2);
plot(alpha2,Fy2,'r');

[alpha2, Fy2, Fz2, ~, ~, gamma2, pi2, testrange2] = TireParser_Cornering(P_input2, 4, FZ_input2);
scatter(alpha2,Fy2);
plot(alpha2,Fy2,'y');
%}

Fyplot2 = lateralforce_pure_test(Xbestcell,alpha2,Fz2,pi2,gamma2);
plot(alpha2,Fyplot2,'k','Linewidth',3);
xlabel('Slip Angle','FontSize',15);
ylabel('Fy:Lateral Force','FontSize',15);
grid on

if plot3 == 1
    Fz3 = -repmat(FZ_input3,numel(alpha3),1);
    pi3 = repmat(P_input3,numel(alpha3),1);
    gamma3 = repmat(IA_input3,numel(alpha3),1);
    
    figure(3)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(FZ_input3)
                Fyplot3 = lateralforce_pure_test(Xbestcell,alpha3,Fz3(:,c),pi3(:,a),gamma3(:,b));
                plot(alpha3,Fyplot3,'k','Linewidth',3);
                hold on
            end
        end
    end
    xlabel('Slip Angle','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
    grid on
end

hold off
if plot2 == 1
    figure(2);
    set(gcf,'Position',[656,194,560,420]);
    plot(1:itermax,errorplot);
    xlabel('Iterations');
    ylabel('Sum-Squared Error');
end


plot4 = 1; %turn on plotting
alpha_input4 = [5 10 15];
P_input4 = [12];
IA_input4 = [0];
FZ4 = linspace(0,300,1000).';

alpha4 = -repmat(FZ_input3,numel(FZ4),1);
    pi4 = repmat(P_input3,numel(FZ4),1);
    gamma4 = repmat(IA_input3,numel(FZ4),1);
    
    figure(4)
    set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input4)
                Fyplot3 = lateralforce_pure_test(Xbestcell,alpha4(:,c),FZ4,pi3(:,a),gamma3(:,b));
                plot(FZ4,Fyplot3,'k','Linewidth',3);
                hold on
            end
        end
    end
    xlabel('Normal Load','FontSize',15);
    ylabel('Fy:Lateral Force','FontSize',15);
    grid on

%% Save Parameters

%save('Fy_pure_parameters.mat','Xbestcell');
