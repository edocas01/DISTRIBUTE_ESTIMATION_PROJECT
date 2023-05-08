clc;
close all;
clear all;

C = [1.5,0.5;0.5,0.5];
mu = [1;3];

[V,D] = eig(C);
theta = atan2(V(2,1),V(1,1));
theta_deg = theta*180/pi;

axis equal;
hold on;
% vector from mu to mu+V(:,1)
plot([mu(1),mu(1)+V(1,1)],[mu(2),mu(2)+V(2,1)],'r','LineWidth',2);
plot([mu(1),mu(1)+V(1,2)],[mu(2),mu(2)+V(2,2)],'r','LineWidth',2);
