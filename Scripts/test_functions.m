clearvars;
clc;
% rng default;
addpath('Classes');
addpath('Functions');

config;
N = parameters_simulation.N;
range = 5;

dyn_type = repmat("linear",N,1);
T = TARGET([0;0]);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

figure(1)
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot_real(all_markers, color_matrix, true);
end

for i = 1:N
	for j = 1:10
    	EKF(R{i}, 0)
	end
end
legend show
relative_general_consensous(R, T, parameters_simulation);

for i = 1:N
    check(:,i) = R{i}.all_robots_pos(:);
end
check

voronoi_map_consensous(parameters_simulation, R, [], 3);

for i = 1:N	
	plot(R{i}.voronoi,'HandleVisibility','off');
end