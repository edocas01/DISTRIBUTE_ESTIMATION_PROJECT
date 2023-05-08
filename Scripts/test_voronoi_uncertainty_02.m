clc;
close all;
clear all;

p1 = [1,3];
p2 = [10,6];
cov_matrix1 = [10, 0.8; 0.8, 7];
cov_matrix2 = [0.1, 0.2; 0.2, 6];
% find the distance between the points and thei covariance ellipses in the direction of the line joining them

figure()
hold on
plot(p1(1),p1(2),'r*');
plot(p2(1),p2(2),'b*');
% plot the line joining the points in green
plot([p1(1),p2(1)],[p1(2),p2(2)],'g');
% plot the extremes 
points(p1,p2,cov_matrix1,cov_matrix2);

axis equal

function plotErrorEllipse(mu, Sigma, p, color,alpha)

    s = -2 * log(1 - p);
    s = p;
    [V, D] = eig(Sigma * s);
    theta = atan2(V(2,1), V(1,1));
    t = theta + pi/2;
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
    plot(a(1, :) + mu(1), a(2, :) + mu(2),'+k','MarkerSize',20);

    t = linspace(0, 2 * pi, 100);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
    plot(a(1, :) + mu(1), a(2, :) + mu(2),color,'MarkerSize',20);
    % plot the eigenvectors
    plot([mu(1), mu(1) + V(1,1)],[mu(2), mu(2) + V(2,1)],'-k');
    plot([mu(1), mu(1) + V(1,2)],[mu(2), mu(2) + V(2,2)],'-k');
end

function points(p1,p2,cov1,cov2)
    dir = p2 - p1;
    alpha = atan2(dir(2),dir(1));
    
    plotErrorEllipse(p1,cov1,1,'r',alpha);
    plotErrorEllipse(p2,cov2,1,'b',alpha);
end