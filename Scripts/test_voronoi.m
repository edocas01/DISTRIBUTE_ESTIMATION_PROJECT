clc;
close all;
clearvars;
config;
N = 10;
range = 2;
figure();
hold on
axis equal
for i = 1:N
	x = cosd(360/N*i)*range;
    y = sind(360/N*i)*range;
	robots{i} = ROBOT([x;y], i, 'linear', parameters_simulation);
    robots{i}.plot(all_markers,color_matrix,false);
end
target = TARGET([0;0]);
plot(target.x(1), target.x(2),'.r')
relative_target_consensous(robots, target, parameters_simulation);
voronoi_map(robots);

for i = 1:N
    plot(robots{i}.voronoi)
end