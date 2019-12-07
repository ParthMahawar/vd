function [SA_out, FY_out, FZ_out, P_out, TSTC_out, ET_out, index] = TireTemperatureParser(P_input, IA_input, FZ_input, SA_input, file_name)

P_tol = .8;
IA_tol = 0.4;
FZ_tol = 5;
SA_tol = 10;

FZ_input = -FZ_input; %convention

load(file_name);

index = [];

%isolate data based on given parameters
for i = 1:numel(P_input)
    for j = 1:numel(IA_input)
        for k = 1:numel(FZ_input)
            for l = 1:numel(SA_input)
            index = [index; find(P > P_input(i) - P_tol & P < P_input(i) + P_tol & IA > IA_input(j) - IA_tol...
            & IA < IA_input(j) + IA_tol & FZ > FZ_input(k) - FZ_tol & FZ < FZ_input(k) + FZ_tol...
            & SA > SA_input(l) - SA_tol & SA < SA_input(l) + SA_tol )];
            end
        end
    end
end

index = index(1:end);

%outputs
SA_out = SA(index);
FY_out = FY(index);
FZ_out = FZ(index);
P_out = P(index);
TSTC_out = TSTC(index);
ET_out = ET(index);
