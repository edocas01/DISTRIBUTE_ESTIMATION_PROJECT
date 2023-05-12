close all
clc;

tic

P = [1,0.5;0.5,10];
c = [1,1];
point = [5,6];
figure()
[points, new_pj] = moving_closer_point(point, c, P, 3);

plot(point(1), point(2), '*r');
hold on
plot(points(1,:), points(2,:), '-b');
plot(c(1), c(2), '*b');
plot(new_pj(1), new_pj(2), '*g');
axis equal
toc 