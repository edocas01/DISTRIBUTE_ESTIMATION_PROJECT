% clc; clear; close all;
clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');

coverage = 3;
config;
N = 10;
range = 5;

dyn_type = repmat("linear",N,1);
T = TARGET([0;0]);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

for i = 1:N
	for j = 1:10
    	EKF(R{i}, 0)
	end
end

relative_target_consensous(R, T, parameters_simulation);
voronoi_map(parameters_simulation, R, [], coverage);

phi = @(x,y) exp(-((x - T.x(1))^2 + (y - T.x(2))^2)); % kg / m^2

tic

figure(1)
T.plot();
hold on; grid on; axis equal;
leg = cell(1,N);
for i = 1:N
	[barycenter, msh] = compute_centroid(R{i}.voronoi, phi);
	h(i) = R{i}.plot(all_markers, color_matrix, false);
	leg{i} = sprintf('Robot %d', i);
	if i == N
	 	h(i+1) = plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
		leg{i+1} = 'Centroid';
	else
		plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off')
	end
	plot(R{i}.voronoi, 'HandleVisibility', 'off')
	pdemesh(msh);
end
legend(h, leg, 'Location', 'bestoutside')
hold off

toc


