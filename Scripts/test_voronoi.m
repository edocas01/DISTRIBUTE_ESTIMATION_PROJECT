clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');
coverage = 3;
coverage_01 = 1;
config;
N = 6;
range = 3;
x = [10,6];
y = [0,15];
for i = 1:N
        x = cosd(360/N*i)*range;
        y = sind(360/N*i)*range;
%     x = (rand() -0.5)*range;
%     y = (rand() -0.5)*range;
	robots{i} = ROBOT([x;y], i, 'linear', parameters_simulation);
% robots{i} = ROBOT([x(i);y(i)], i, 'linear', parameters_simulation);
end
target = TARGET([0;0]);

for i = 1:N
    for j = 1:10
        EKF(robots{i}, 0)
    end
end

relative_target_consensous(robots, target, parameters_simulation);
robots_01 = robots;
voronoi_map(parameters_simulation, robots, [], coverage);


figure(1);
subplot(1, 2, 1)
hold on
axis equal

for i = 1:N
    robots{i}.plot(all_markers,color_matrix,false);
    plot(robots{i}.voronoi)
    pp = compute_ellipse(robots{i}.x_est, robots{i}.P, coverage);
    plot(pp(1,:), pp(2,:),'k');
    [pointsx,pointsy] = Circle(robots{i}.x_est(1),robots{i}.x_est(2), robots{i}.volume);
    poly_circle = polyshape(pointsx,pointsy);
    plot(poly_circle,'FaceColor','black')
end
pp = compute_ellipse(robots{i}.target_est, robots{i}.target_P, coverage);
plot(pp(1,:), pp(2,:),'k');
plot(robots{i}.target_est(1), robots{i}.target_est(2),'.r')


voronoi_map(parameters_simulation, robots_01, [], coverage_01);

figure(1);
subplot(1, 2, 2)
hold on
axis equal

for i = 1:N
    robots_01{i}.plot(all_markers,color_matrix,false);
    plot(robots_01{i}.voronoi)
    pp = compute_ellipse(robots_01{i}.x_est, robots_01{i}.P, coverage_01);
    plot(pp(1,:), pp(2,:),'k');
    [pointsx,pointsy] = Circle(robots_01{i}.x_est(1),robots_01{i}.x_est(2), robots_01{i}.volume);
    poly_circle = polyshape(pointsx,pointsy);
    plot(poly_circle,'FaceColor','black')
end
pp = compute_ellipse(robots_01{i}.target_est, robots_01{i}.target_P, coverage_01);
plot(pp(1,:), pp(2,:),'k');
plot(robots_01{i}.target_est(1), robots_01{i}.target_est(2),'.r')
