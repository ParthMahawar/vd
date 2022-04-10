clc
clear

%% Dimensions
WB = 61; % in, wheelBase
TW = 47; % in, trackWidth

FW = 24; % in, Front Wing Overhang

p.RR = [0,0,0]; % RR
p.RL = [0,TW,0]; % RL
p.FR = [WB,0,0]; % FR
p.FL = [WB,TW,0]; % FL
p.FRW = [WB+FW,0,0]; % FR-Wing
p.FLW = [WB+FW,TW,0]; % FL-Wing

%% Calc Points

% referenced from RR
p.RL(3) = calculatePosition(p.RR, p.RL, 0.00, 0);
p.FR(3) = calculatePosition(p.RR, p.FR, 0, 0.00);
p.FRW(3) = calculatePosition(p.RR, p.FRW, 0, 0.10);

% referenced from RL
p.FL(3) = calculatePosition(p.RL, p.FL, 0, -0.50);
p.FLW(3) = calculatePosition(p.RL, p.FLW, 0, -0.50);

% referenced from FR
%p.FL(3) = calculatePosition(p.FR, p.FL, -0.20, 0);

% check accuracy
angle1 = calcAngle(p.FL, p.FR) % should equal -0.2
angle2 = calcAngle(p.FLW, p.FRW) % should equal 0.5
%%
calcAngle(p.RR, p.FRW)

%% Plotting
close all
hold on
plotPoint(p, 'RR');
plotPoint(p, 'RL');
plotPoint(p, 'FR');
plotPoint(p, 'FL');
plotPoint(p, 'FRW');
plotPoint(p, 'FLW');
legend('AutoUpdate','off');

x = repmat([p.RR(1) p.FR(1) p.FRW(1)],2,1);
y = repmat([p.RR(2); p.RL(2)],1,3);
z = [p.RR(3) p.FR(3) p.FRW(3);...
     p.RL(3) p.FL(3) p.FLW(3)];

surf(x,y,z);


ylabel('y (in)');
xlabel('x (in)');
%axis padded

xlim([-10, 100])
ylim([-10, 100])
zlim([-10, 10])



%% Functions

function z = calculatePosition(point1, point2, angleX, angleY)
    % angles in degrees, with coordinates, calculates z of point 2
    delta = point2-point1;
    z_a = delta(2)*deg2rad(angleX);
    z_b = -delta(1)*deg2rad(angleY);
    z = z_a + z_b + point1(3);
end

function varargout = plotPoint(p, name, varargin)
    point = p.(name);
    scatter3(point(1), point(2), point(3), ...
        'displayName', name, 'MarkerFaceColor','auto');
%     varargout{1:nargout} = scatter3(point(1), point(2), point(3),...
%         'displayName', name, varargin{:});
end

function angle = calcAngle(point1, point2)
    d = norm(point1-point2);
    dz = point1(3)-point2(3);

    angle = rad2deg(dz/d);
end
