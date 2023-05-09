clc; clear; 
addpath('Functions')

P1 = [2, 0.3;
     0.3, 3]; % Covariance matrix of 

P2 = [10, 0.2;
     0.2, 4];

c1 = [1; 3];
[pts1, rx1, ry1, th1] = ellipse(c1, P1);
Rth1 = [cos(th1), -sin(th1);
        sin(th1), cos(th1)];

% All possible relative ellipse's relative positions        
c2 = [7; 8];
c2 = [-7; 8]; 
c2 = [-7;-8]; 
c2 = [7; -8]; 

[pts2, rx2, ry2, th2] = ellipse(c2, P2);
Rth2 = [cos(th2), -sin(th2);
        sin(th2), cos(th2)];


% Ellipse 1
c2_in1 = Rth1' * (c2 - c1);
dir = c2_in1;
alpha = mod(atan2(dir(2), dir(1)), 2 * pi); % Real angle at which c2 is seen by c1
phi = atan(rx1 / ry1 * tan(alpha)); % Fake angle to get the correct radius
% dist = sqrt(rx1^2 * cos(phi)^2 + ry1^2 * sin(phi)^2);
gain = 1;
pt_inter1 = c1 + Rth1 * gain * [rx1 * cos(phi); ry1 * sin(phi)];

norm(c1 - pt_inter1)
sqrt((rx1 * cos(phi))^2 + (ry1 * sin(phi))^2)

% Ellipse 2
c1_in2 = Rth2' * (c1 - c2);
dir = c1_in2;
alpha = mod(atan2(dir(2), dir(1)), 2 * pi); % Real angle at which c1 is seen by c2
phi = atan(rx2 / ry2 * tan(alpha)); % Fake angle to get the correct radius
dist = sqrt(rx2^2 * cos(phi)^2 + ry2^2 * sin(phi)^2);
gain = 1;
pt_inter2 = c2 + Rth2 * gain * dist * [cos(alpha); sin(alpha)];

figure(1); clf; axis equal; grid on
xlim("padded")
ylim("padded")
hold on
plot(c1(1), c1(2), 'r*')
plot(c2(1), c2(2), 'b*')
plot([c1(1), c2(1)], [c1(2), c2(2)], 'k-')
plot(pts1(1,:), pts1(2,:), 'r-')
plot(pts2(1,:), pts2(2,:), 'b-')
plot(pt_inter1(1), pt_inter1(2), 'k*')
plot(pt_inter2(1), pt_inter2(2), 'k*')
plot([c1(1), pt_inter1(1)], [c1(2), pt_inter1(2)], 'r-')
plot([c2(1), pt_inter2(1)], [c2(2), pt_inter2(2)], 'b-')
hold off



