clc; clear; 
addpath('Functions')

P1 = [0.5, 0.3;
     0.3, 0.5];
P2 = [0.5, 0.3;
     0.3, 0.5];

c1 = [1; -2];
[pts1, rx1, ry1, th1] = ellipse(c1, P1);
Rth1 = [cos(th1), -sin(th1);
        sin(th1), cos(th1)];

% All possible relative ellipse's relative positions        
c2 = [7; 8]; 
c2 = [-7; 8]; 
c2 = [-7;-8]; 
% c2 = [7; -8]; 

% BISOGNA SOLO CAPIRE QUANDO AGGIUNGERE PI A PHI E QUANDO NO 
% IN BASE A COME SONO MESSE LE ELLISSI 

[pts2, rx2, ry2, th2] = ellipse(c2, P2);
Rth2 = [cos(th2), -sin(th2);
        sin(th2), cos(th2)];

dir = c2 - c1;
alpha = mod(atan2(dir(2), dir(1)), pi);

ang = (alpha - th1);
phi = atan(rx1 / ry1 * tan(ang));
phi = phi;
if c1(1) > c2(1) && alpha - th1 > pi/2
    phi = phi + pi;
end

gain = 1;
pt_inter1 = c1 + Rth1 * gain * [rx1 * cos(phi); ry1 * sin(phi)];

ang = alpha - th2;
phi = atan(rx2 / ry2 * tan(ang));
phi = phi + pi;
if c2(1) < c1(1) && alpha - th2 > pi/2
    phi = phi + pi;
end

gain = 1;
pt_inter2 = c2 + Rth2 * gain * [rx2 * cos(phi); ry2 * sin(phi)];

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
hold off


