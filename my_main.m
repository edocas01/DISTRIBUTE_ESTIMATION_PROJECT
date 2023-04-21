% Syntax functions
% Robot_cell_array = select_shape(N_robots, type_dynamics, shape, center_point, distance, randdistance, param)

clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

%% ----------------------- %
%  Define default settings %
%  ----------------------- %
config;
parameters_simulation

N = 10; % Number of robots
if N > parameters_simulation.N_MAX
	warning('Too many robots. There will be created %d robots', parameters_simulation.N_MAX);
end

%% ---------------------- %
%  Initialize environment %
%  ---------------------- %
type_dynamics = repmat("linear", 1, N);
dist = 1;
center_point = [0, 0];
R = select_shape(N, type_dynamics, "square", center_point, dist, true, parameters_simulation);
[target, u_trajectory, obstacles] = initialize_env(parameters_simulation);


% ---------------------
% Initial configuration
figure(1); clf;
hold on; grid on;
axis equal;

xlim("padded")
ylim("padded")
for i = 1:N
	R{i}.plot(all_markers, color_matrix, false);
end
target.plot();
for i = 1:length(obstacles)
	obstacles{i}.plot();
end
hold off;


%% ----------------------------------- %
%  Static localization with consensous %
%  ----------------------------------- %
for i = 1:N
	R{i}.Clear_Targ_Estimates();
end
relative_target_consensous(R, target, parameters_simulation)

% Estimation error 
figure(2); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	errx = R{i}.target_est_hist(1,:) - target.x(1);
	plot(errx, 'Color', color_matrix(i,:));
end
xlabel("Consensous iteration")
ylabel("Error in x (m)")
hold off;

figure(3); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	erry = R{i}.target_est_hist(2,:) - target.x(2);
	plot(erry, 'Color', color_matrix(i,:));
end
xlabel("Consensous iteration")
ylabel("Error in y (m)")
hold off;

%% -------------------------------------------------- %
%  Static localization with distributed Kalman filter %
%  -------------------------------------------------- %
for i = 1:N
	R{i}.Clear_Targ_Estimates();
end
distributed_KF(R, target, parameters_simulation);

% Estimated target position
figure(4); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	plot(R{i}.target_est(1), R{i}.target_est(2), 'o', 'Color', color_matrix(i,:));
end
target.plot()
xlabel("x (m)")
ylabel("y (m)")
hold off;





