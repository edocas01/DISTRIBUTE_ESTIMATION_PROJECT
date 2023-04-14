clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

% Load config
config;
N = 10;
if N > parameters_simulation.N_MAX
    N = parameters_simulation.N_MAX;
end


%% Test robot
% robot needs x0,y0,comradius,id,type
robot = ROBOT(0,0,0.1,1,'linear');
target = TARGET(0,0);
u_target = target_trajectory(parameters_simulation);

% time = 0:parameters_simulation.dt:parameters_simulation.tmax;
time = 0:parameters_simulation.dt:length(u_target(:,1));
for t = 1:length(u_target(:,1))
	u = [0.001;0.001];
	EKF(robot, u);

	err(t,:) = norm(robot.x - robot.x_est);
    x_true(t,:) = robot.x;
    x_est(t,:) = robot.x_est;

    target.dynamics(u_target(t,:));
    traj(t,:) = target.x;

end


plot(x_true(:,1),x_true(:,2),'-k')
hold on
plot(x_est(:,1),x_est(:,2),'-b')
hold on
plot(traj(:,1),traj(:,2),'-r')



