clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');

config;
N = 10;
range = 20;
for i = 1:N
    % 	x = cosd(360/N*i)*range;
    %   y = sind(360/N*i)*range;
    x = (rand() -0.5)*range;
    y = (rand() -0.5)*range;
	robots{i} = ROBOT([x;y], i, 'linear', parameters_simulation);
end
target = TARGET([0;0]);

for i = 1:N
    EKF(robots{i}, 0)
end

relative_target_consensous(robots, target, parameters_simulation);
voronoi_map(robots);

figure();
hold on
axis equal
plot(target.x(1), target.x(2),'.r')
for i = 1:N
    robots{i}.plot(all_markers,color_matrix,false);
    plot(robots{i}.voronoi)
end