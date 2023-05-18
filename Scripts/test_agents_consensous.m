clc; clear; close all;

config;
% [T, trajectory, u_trajectory, obstacles] = initialize_env(parameters_simulation);


N = 5;
range = 5;
T = TARGET([0;0]);
fprintf("Target initial position: (%.2f m, %.2f m)\n", T.x(1), T.x(2));

dyn_type = repmat("linear",N,1);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

figure(1); clf
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot_est(all_markers, color_matrix, true);
end
legend show
hold off


[z, cov] = relative_general_consensous(R, T, parameters_simulation);