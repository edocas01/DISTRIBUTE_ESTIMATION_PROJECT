%% Syntax functions
% Robot_cell_array = select_shape(N_robots, type_dynamics, shape, center_point, distance, randdistance, param)

clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

%% ----------------------- %
%  DEFINE DEFAULT SETTINGS %
%  ----------------------- %
config;
parameters_simulation

N = 10; % Number of robots
if N > parameters_simulation.N_MAX
	warning('Too many robots. There will be created %d robots', parameters_simulation.N_MAX);
end

%% ---------------------- %
%  INITIALIZE ENVIRONMENT %
%  ---------------------- %
type_dynamics = repmat("linear", 1, N);
dist = 1;
center_point = [0, 0];
R = select_shape(N, type_dynamics, "square", center_point, dist, true, parameters_simulation);
[target, trajectory, u_trajectory, obstacles] = initialize_env(parameters_simulation);


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
plot(trajectory(1,:), trajectory(2,:), '--r', 'LineWidth', 2);
hold off;


%% -------------------------------------------------- %
%  STATIC LOCALIZATION WITH MAXIMUM DEGREE CONSENSOUS  %
%  -------------------------------------------------- %
target.Initialize_Position();
for i = 1:N
	R{i}.Clear_Targ_Estimates();
	R{i}.Clear_Targ_Estimates_Hist();
	R{i}.Initialize_Position();
end
relative_target_consensous(R, target, parameters_simulation)

% Estimation error 
figure(2); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	errx = R{i}.target_est_hist_messages(1,:) - target.x(1);
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
	erry = R{i}.target_est_hist_messages(2,:) - target.x(2);
	plot(erry, 'Color', color_matrix(i,:));
end
xlabel("Consensous iteration")
ylabel("Error in y (m)")
hold off;

%% -------------------------------------------------- %
%  STATIC LOCALIZATION WITH DISTRIBUTED KALMAN FILTER %
%  -------------------------------------------------- %
% Ther robots and the target don't move
target.Initialize_Position();
for i = 1:N
	R{i}.Clear_Targ_Estimates();
	R{i}.Clear_Targ_Estimates_Hist();
	R{i}.Initialize_Position();
end
distributed_KF(R, target, parameters_simulation);

% Estimated target position
figure(4); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	errx = abs(R{i}.target_est(1) - target.x(1));
	stem(R{i}.id, errx, 'Color', color_matrix(i,:));
end
xlabel("Robot")
ylabel("$|$ Error in x $|$ (m)")
hold off;

figure(5); clf;
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	erry = abs(R{i}.target_est(2) - target.x(2));
	stem(R{i}.id, erry, 'Color', color_matrix(i,:));
end
xlabel("Robot")
ylabel("$|$ Error in y $|$ (m)")

%% ---------------------------------------------------------- %
%  ESTIMATION OF MOVING TARGET WITH MAXIMUM DEGREE CONSENSOUS %
%  ---------------------------------------------------------- %
% The robots don't move, only the target moves.
target.Initialize_Position();
for i = 1:N
	R{i}.Clear_Targ_Estimates();
	R{i}.Clear_Targ_Estimates_Hist();
	R{i}.Initialize_Position();
end
time = length(u_trajectory);
traj_step = zeros(2, length(u_trajectory));
traj_step(:,1) = target.x;
for j = 1:N
	R{j}.target_est_hist(:,1) = R{j}.target_est;
end

for i = 2:time 
	% Update target position
	target.dynamics(u_trajectory(:,i));
	traj_step(:,i) = target.x;
	% Update target estimation
	relative_target_consensous(R, target, parameters_simulation);
	% Update history of target estimation
	for j = 1:N
		R{j}.target_est_hist(:,i) = R{j}.target_est;
	end
	
end

% -------------------------------
% Estimated trajectory and errors
figure(6); clf;
subplot(3,1,1)
hold on; grid on;
xlim("padded")
ylim("padded")
title("Trajectory estimation with maximum degree consensous")
for i = 1:N
	plot(R{i}.target_est_hist(1,:), R{i}.target_est_hist(2,:), 'Color', color_matrix(i,:));
end
plot(trajectory(1,:), trajectory(2,:), '--r', 'LineWidth', 2);
xlabel("x (m)")
ylabel("y (m)")
hold off;

subplot(3,1,2)
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	errx = R{i}.target_est_hist(1,:) - traj_step(1,:);
	plot(errx, 'Color', color_matrix(i,:));
end
xlabel("Time (s)")
ylabel("Error in x (m)")
hold off;

subplot(3,1,3)
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	erry = R{i}.target_est_hist(2,:) - traj_step(2,:);
	plot(erry, 'Color', color_matrix(i,:));
end
xlabel("Time (s)")
ylabel("Error in y (m)")
hold off;


%% ---------------------------------------------------------- %
%  ESTIMATION OF MOVING TARGET WITH DISTRIBUTED KALMAN FILTER %
%  ---------------------------------------------------------- %
% The robots don't move, only the target moves.
target.Initialize_Position();
for i = 1:N
	R{i}.Clear_Targ_Estimates();
	R{i}.Clear_Targ_Estimates_Hist();
	R{i}.Initialize_Position();
end
time = length(u_trajectory);
traj_step = zeros(2, length(u_trajectory));
traj_step(:,1) = target.x;
for j = 1:N
	R{j}.target_est_hist(:,1) = R{j}.target_est;
end

for i = 2:time 
	% Update target position
	target.dynamics(u_trajectory(:,i));
	traj_step(:,i) = target.x;
	% Update target estimation
	distributed_KF(R, target, parameters_simulation);
	% Update history of target estimation
	for j = 1:N
		R{j}.target_est_hist(:,i) = R{j}.target_est;
	end

end

% --------------------
% Estimated trajectory

figure(7); clf;
subplot(3,1,1)
hold on; grid on;
xlim("padded")
ylim("padded")
title("Trajectory estimation with distributed Kalman filter")
for i = 1:N
	plot(R{i}.target_est_hist(1,:), R{i}.target_est_hist(2,:), 'Color', color_matrix(i,:));
end
plot(trajectory(1,:), trajectory(2,:), '--r', 'LineWidth', 2);
xlabel("x (m)")
ylabel("y (m)")
hold off;

subplot(3,1,2)
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	errx = R{i}.target_est_hist(1,:) - traj_step(1,:);
	plot(errx, 'Color', color_matrix(i,:));
end
xlabel("Time (s)")
ylabel("Error in x (m)")
hold off;

subplot(3,1,3)
hold on; grid on;
xlim("padded")
ylim("padded")
for i = 1:N
	erry = R{i}.target_est_hist(2,:) - traj_step(2,:);
	plot(erry, 'Color', color_matrix(i,:));
end
xlabel("Time (s)")
ylabel("Error in y (m)")
hold off;




