clc;
close all;
clear all;

p1 = [1,3];
p2 = [10,8];
cov_matrix1 = [1, 0.2; 0.2, 1];
cov_matrix2 = [0.9, 0.1; 0.1, 0.9];

[p1_t,p2_t] = find_dist(p1,p2,cov_matrix1,cov_matrix2,0.99);
figure();
hold on;
plotErrorEllipse(p1, cov_matrix1, 0.99, 'r');
plot(p1(1),p1(2),'r*');
plotErrorEllipse(p2, cov_matrix2, 0.99, 'b');
plot(p2(1),p2(2),'b*');
plot([p1(1),p2(1)],[p1(2),p2(2)],'-g')
plot(p1_t(1),p1_t(2),'r*');
plot(p2_t(1),p2_t(2),'b*');

% find the distance between the point and its ellipse in the direction of the other point 
function [p1_t,p2_t] = find_dist(p1,p2,cov1,cov2,confidence)
    s = -2 * log(1 - confidence);

    dir = p2 - p1;
    dir = dir/norm(dir);

    angle = atan2(dir(2), dir(1));

    delta1 = (V1 * sqrt(D1)) * [cos(angle); sin(angle)];
    delta2 = (V2 * sqrt(D2)) * [cos(angle); sin(angle)];

    p1_t = p1 + delta1';
    p2_t = p2 - delta2';

end


function plotErrorEllipse(mu, Sigma, p, color)

    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
 
    plot(a(1, :) + mu(1), a(2, :) + mu(2),color);
end
