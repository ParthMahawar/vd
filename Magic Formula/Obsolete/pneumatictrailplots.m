setup_paths

load('Mz_pure_parameters_run24_9.mat')

alpha_input3 = [5];
P_input3 = [12];
IA_input3 = [0];
FZ3 = linspace(50,250,1000).';

alpha3 = repmat(alpha_input3,numel(FZ3),1);
pi3 = repmat(P_input3,numel(FZ3),1);
gamma3 = repmat(IA_input3,numel(FZ3),1);

for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mz_Fycell3 = Mz_lateralforce_pure(Xbestcell_Fy,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b));
            end
        end
    end
    
figure(1)
hold on
set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mzplot3 = 12 * selfaligningmoment_pure(Xbestcell,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b),Mz_Fycell3);
                plot(FZ3,Mzplot3,'k','Linewidth',3);
            end
        end
    end
    xlabel('Normal Load (lb)','FontSize',15');
    ylabel('Peak Pneumatic Trail (in)','FontSize',15');
    grid on

    %{
load('Mz_pure_parameters_run27_6.mat')

    
alpha_input3 = [5];
P_input3 = [12];
IA_input3 = [0];
FZ3 = linspace(50,250,1000).';

alpha3 = repmat(alpha_input3,numel(FZ3),1);
pi3 = repmat(P_input3,numel(FZ3),1);
gamma3 = repmat(IA_input3,numel(FZ3),1);

for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mz_Fycell3 = Mz_lateralforce_pure(Xbestcell_Fy,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b));
            end
        end
    end
    
set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mzplot3 = 12 * selfaligningmoment_pure(Xbestcell,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b),Mz_Fycell3);
                plot(FZ3,Mzplot3,'k','LineStyle','--','Linewidth',3);
            end
        end
    end
    xlabel('Normal Load (lb)','FontSize',15');
    ylabel('Peak Pneumatic Trail (in)','FontSize',15');
    grid on
    
alpha_input3 = [5];
P_input3 = [12];
IA_input3 = [0];
FZ3 = linspace(50,250,1000).';

load('Mz_pure_parameters_run29_5.mat')


alpha3 = repmat(alpha_input3,numel(FZ3),1);
pi3 = repmat(P_input3,numel(FZ3),1);
gamma3 = repmat(IA_input3,numel(FZ3),1);

for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mz_Fycell3 = Mz_lateralforce_pure(Xbestcell_Fy,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b));
            end
        end
    end
    
set(gcf,'Position',[656,194,560,420]);
    for a = 1:numel(P_input3)
        for b = 1:numel(IA_input3)
            for c = 1:numel(alpha_input3)
                Mzplot3 = 12 * selfaligningmoment_pure(Xbestcell,alpha3(:,c),FZ3,pi3(:,a),gamma3(:,b),Mz_Fycell3);
                plot(FZ3,Mzplot3,'k','LineStyle',':','Linewidth',3);
            end
        end
    end
    xlabel('Normal Load (lb)','FontSize',15');
    ylabel('Peak Pneumatic Trail (in)','FontSize',15');
    grid on
    
 hold off
    %}
    
title('Peak Pneumatic Trail vs Normal Load','FontSize',18);
leg = legend('18.0X7.5-10, 8 in rims,12 psi');
leg.Position = [0.3 0.7 0.1 0.2];
leg.FontSize = 12;
