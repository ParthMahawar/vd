function [SR_out, SA_out, FX_out, FY_out, FZ_out, MZ_out, IA_out, P_out, index] = TireParser_DriveBrake(P_input, IA_input, FZ_input,SA_input, file_name)

P_tol = 0.5;
IA_tol = 0.1;
FZ_tol = 10;
SA_tol = 0.1;

FZ_input = -FZ_input; %convention

load(file_name);

index = [];

%isolate data based on given parameters
for i = 1:numel(P_input)
    for j = 1:numel(IA_input)
        for k = 1:numel(FZ_input)
            for l = 1:numel(SA_input)
                index = [index; find(P > P_input(i) - P_tol & P < P_input(i) + P_tol & IA > IA_input(j)...
                    - IA_tol & IA < IA_input(j) + IA_tol & FZ > FZ_input(k) - FZ_tol & FZ < FZ_input(k)...
                    + FZ_tol & SA > SA_input(l) - SA_tol & SA < SA_input(l) + SA_tol)];
            end
        end
    end
end

index = index(1:10:end);

%index = index(find(SL(index)>0));

%outputs
SR_out = SL(index);
SA_out = SA(index);
FX_out = FX(index);
FY_out = FY(index);
FZ_out = FZ(index);
MZ_out = MZ(index);
IA_out = IA(index);
P_out = P(index);