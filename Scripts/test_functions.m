clearvars;
clc;
% rng default;
addpath('Classes');
addpath('Functions');

config;
N = 8;
range = 15;

dyn_type = repmat("linear",N,1);
T = TARGET([0;0]);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

figure(1)
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot(all_markers, color_matrix, false);
end
hold off
for i = 1:N
	for j = 1:10
    	EKF(R{i}, 0)
	end
end


[z, cov] = relative_general_consensous(R, T, parameters_simulation);