% winning times (based on 2019 Lincoln)
skidpad_winning_time = 5.031;
accel_winning_time = 3.993;
autocross_winning_time = 58.383;
endurance_winning_time = 1344.592;

% skidpad
t_your = linspace(5.031,6);
t_min = skidpad_winning_time;
t_max = 1.25 * t_min*ones(size(t_your));
if t_your > t_max
   skidpad_points = 3.5;
else
   skidpad_points =  71.5 * ((t_max ./ t_your).^2.0 - 1.0) ./ ((t_max ./ t_min).^2.0 - 1.0) + 3.5;
end

plot(t_your(1:end-1),-diff(skidpad_points)./diff(t_your))
xlabel('Time (s)')
ylabel('Skidpad Points per Second')

%%
% accel
t_your = linspace(3.993,5.5);
t_min = accel_winning_time;
t_max = t_min * 1.5*ones(size(t_your));
if t_your > t_max
    accel_points = 4.5;
else
    accel_points = 95.5 * ((t_max ./ t_your) - 1.0) ./ ((t_max / t_min) - 1.0) + 4.5;
end

plot(t_your(1:end-1),-diff(accel_points)./diff(t_your))
xlabel('Time (s)')
ylabel('Accel Points per Second')

%%

% autocross
t_your = linspace(58.383,70);
t_min = autocross_winning_time;
t_max = 1.45 * t_min*ones(size(t_your));
if t_your > t_max
    autocross_points = 6.5;
else
    autocross_points = 118.5 * ((t_max ./ t_your) - 1.0) ./ ((t_max ./ t_min) - 1.0) + 6.5;
end

plot(t_your(1:end-1),-diff(autocross_points)./diff(t_your))
xlabel('Time (s)')
ylabel('Autocross Points per Second')

%%

% endurance
t_your = linspace(1344.592,1600);
t_min = endurance_winning_time;
t_max = 1.45 * t_min*ones(size(t_your));
if t_your > t_max
    endurance_points = 25;
else 
    endurance_points = 200 * ((t_max ./ t_your) - 1.0) ./ ((t_max ./ t_min) - 1.0) + 25.0;
end

plot(t_your(1:end-1),-diff(endurance_points)./diff(t_your))
xlabel('Time (s)')
ylabel('Endurance Points per Second')

%%
points = struct();
points.skidpad = skidpad_points;
points.accel = accel_points;
points.autocross = autocross_points;
points.endurance = endurance_points;
obj.points = points;
obj.totalPoints = sum([skidpad_points; 
                       accel_points;
                       autocross_points;
                       endurance_points]);