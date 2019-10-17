function [SA_out, FY_out, FZ_out, MZ_out, MX_out, IA_out, P_out, index] = TireParser_Cornering(P_input, IA_input, FZ_input)

P_tol = .8;
IA_tol = 0.4;
FZ_tol = 5;

FZ_input = -FZ_input; %convention

%load('A1654run25.mat');
load('A1965run15.mat');

index = [];

%isolate data based on given parameters
for i = 1:numel(P_input)
    for j = 1:numel(IA_input)
        for k = 1:numel(FZ_input)
            index = [index; find(P > P_input(i) - P_tol & P < P_input(i) + P_tol & IA > IA_input(j) - IA_tol...
            & IA < IA_input(j) + IA_tol & FZ > FZ_input(k) - FZ_tol & FZ < FZ_input(k) + FZ_tol)];
        end
    end
end

index = index(1:10:end);

%outputs
SA_out = SA(index);
FY_out = FY(index);
FZ_out = FZ(index);
MZ_out = MZ(index);
MX_out = MX(index);
IA_out = IA(index);
P_out = P(index);