clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

% Syntax classes
% ROBOT(x, y, comradius, id, type, param)      type = 'linear' or 'unicycle'
% ROBOT.dynamics(u)                     u = input vector
% ROBOT.plot(all_markers)
% TARGET(x,y)


% Load config
config;
N = 3;
if N > parameters_simulation.N_MAX
    N = parameters_simulation.N_MAX;
end

%% Initialize robots
robots = cell(1,N);
for i = 1:N
    robots{i} = ROBOT([cosd(360/N*i); sind(360/N*i)], i, 'linear', parameters_simulation);
end
[target, u_target, obstacles] = initialize_env(parameters_simulation);
time = 0 : parameters_simulation.dt : length(u_target);
for t = 1:length(u_target)
    target.dynamics(u_target(:,t));
    traj(:,t) = target.x;
    relative_target_consensous(robots,target,parameters_simulation);
    traj_est(:,t,1) = robots{1}.target_est;
    traj_est(:,t,2) = robots{2}.target_est;
end
plot(traj(1,:),traj(2,:),'r');
hold on
plot(traj_est(1,:,1),traj_est(2,:,1),'b');
hold on
plot(traj_est(1,:,2),traj_est(2,:,2),'g');
plot_obstacles(obstacles);
for i = 1:N
    robots{i}.plot(all_markers)
    hold on
end
% 
% target = TARGET([1;4]);
% target.plot()
% legend
% relative_target_consensous(robots,target,parameters_simulation);

% disp(robots{1}.target_est);
% disp(robots{1}.target_P);
% disp(robots{3}.target_est);
% disp(robots{3}.target_P);
% for i = 1:N
%     robots{i}.plot(all_markers)
%     hold on
% end
% target.plot()

% min_map_width = -parameters_simulation.size_map;
% max_map_width = parameters_simulation.size_map;
% min_map_height = -parameters_simulation.size_map;
% max_map_height = parameters_simulation.size_map;
% ComRadius = 0.5;

% %% Initialize robots
% robots = cell(1,N);
% for i = 1:N
%     % This should set x and y in -50 and 50 but it doesn't (to be solved)
%     xtmp = (max_map_width - min_map_width) * rand() + min_map_width;
%     ytmp = (max_map_height - min_map_height) * rand() + min_map_height;
%     % intialize each robot in a random point of the map 
%     robot = ROBOT(xtmp, ytmp, ComRadius, i, 'linear');
%     robots{i} = robot;
% end
% % Assign trajectory
% [target, u_target, obstacles] = initialize_env(parameters_simulation);

% %%
% % time = 0:parameters_simulation.dt:parameters_simulation.tmax;
% time = 0 : parameters_simulation.dt : length(u_target);
% x_true = zeros(2, length(u_target));
% x_est = zeros(2, length(u_target));
% traj = zeros(2, length(u_target));

% for t = 1:length(u_target)
%     target.dynamics(u_target(:,t));
    
%     for i = 1:N
%         EKF(robots{i}, u_target(:,t));
%     end

% 	% err(t,:) = norm(robot.x - robot.x_est);
%     traj(:,t) = target.x;

% end

% figure(2); clf
% axis(parameters_simulation.size_map*[-1 1 -1 1])
% hold on
% grid on
% axis
% plot(traj(1,:),traj(2,:),'-r')
% for i = 1:N
%     plot(robots{i}.x_est(1,:),robots{i}.x_est(2,:),'ok')
%     robots{i}.plot(all_markers)
% end
% for i = 1:length(obstacles)
%     plot(obstacles{i}.x(1),obstacles{i}.x(2),'sk')
% end
% hold off

% %% This has to be done before updating dynamics of the robots
% figure(3); clf
% [vx, vy] = voronoi_map(robots, obstacles);
% hold on;
% plot (vx, vy, "r");
% hold off
