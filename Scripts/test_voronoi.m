clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');

config;
N = 2;
range = 5.5;
x = [3,6];
y = [0,3];
for i = 1:N
%         x = cosd(360/N*i)*range;
%         y = sind(360/N*i)*range;
    % x = (rand() -0.5)*range;
    % y = (rand() -0.5)*range;
% 	robots{i} = ROBOT([x;y], i, 'linear', parameters_simulation);
robots{i} = ROBOT([x(i);y(i)], i, 'linear', parameters_simulation);
end
target = TARGET([0;0]);

for i = 1:N
    for j = 1:1
        EKF(robots{i}, 0)
    end
end

relative_target_consensous(robots, target, parameters_simulation);
voronoi_map(parameters_simulation, robots);

figure();
hold on
axis equal
plot(target.x(1), target.x(2),'.r')
for i = 1:N
    robots{i}.plot(all_markers,color_matrix,false);
    plot(robots{i}.voronoi)
    pp = compute_ellipse(robots{i}.x_est, robots{i}.P, 3);
    plot(pp(1,:), pp(2,:),'k');
end
pp = compute_ellipse(robots{i}.target_est, robots{i}.target_P, 3);
plot(pp(1,:), pp(2,:),'k');