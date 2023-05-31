%% Syntax functions
% Robot_cell_array = select_shape(N_robots, type_dynamics, shape, center_point, distance, randdistance, param)

clc;
clear;
close all;
clearvars;
rng default;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

% ------------------------ %
%  DEFINE DEFAULT SETTINGS %
%  ----------------------- %
config;
parameters_simulation;


[T,~,u_traj,~] = initialize_env(parameters_simulation);
test_control
fprintf("Target initial position: (%.2f m, %.2f m)\n", T.x(1), T.x(2));
N = parameters_simulation.N;
range = 10;

dyn_type = repmat("linear",N,1);
R = select_shape(N, dyn_type, 'circle', [0;0], range, 0, parameters_simulation);

figure(1); clf
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot_real(all_markers, color_matrix, true);
end
hold off
pause(1)
%% Calculations	
tic
results = run_simulation(R, T, u_traj, parameters_simulation);
toc

%% Animation
tic
figure(2);

for t = 1:length(results)
    clf
	hold on; grid on; 
	xlim([-40 40]); ylim([-40 40]);
	datas = results{t};
	datas.T.plot()
	plot(datas.circle_target(1,:), datas.circle_target(2,:), 'b--', 'LineWidth', 1.5);
	for i = 1:N
		datas.R{i}.plot_real(all_markers, color_matrix, false);
		plot(datas.R{i}.voronoi);
		plot(datas.barycenter(1,i), datas.barycenter(2,i), 'kx', 'LineWidth', 1);
    end
    drawnow
end

toc