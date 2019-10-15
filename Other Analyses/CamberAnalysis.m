%% Kinematic Parameters
roll_camber = 0.45; % deg w/ 1 deg roll
ride_camber = 0.221; % deg w/ 1 in droop
steer_camber = -1.576; % deg at full steer

roll_gradient = [0.35 0.89]; %0.67; % deg/g range (min to max)

static_camber = -0.5; % deg

camber_compliance_outside = 0.5; % deg/g (outside wheel)

cornering_g = 1.5; % g

%% Calculations

roll = roll_gradient*cornering_g;
camber_fromroll = roll_camber*roll;

camber_fromcompliance = cornering_g*camber_compliance_outside;

steer_angle = linspace(0,25,1000);
for i = 1:numel(steer_angle)
    camber_fromsteer = steer_angle(i)/25*steer_camber;
    total_camber(:,i) = static_camber+camber_fromroll+camber_fromcompliance+camber_fromsteer;
end
plot(steer_angle,total_camber(1,:))
hold on
plot(steer_angle,total_camber(2,:))
xlabel('Steer Angle')
ylabel('Outside Wheel Camber (deg)')
legend('Min Roll Gradient','Max Roll Gradient')
title('Cornering Camber at 1.5 g')
