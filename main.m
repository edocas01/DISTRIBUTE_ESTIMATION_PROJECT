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
robot = ROBOT(0,0,0.1,1);

time = 0:parameters_simulation.dt:parameters_simulation.tmax;
for t = 1:length(time)-1

	u = [cosd(t); sind(t)];
	EKF(robot, u);

	err(t,:) = norm(robot.x - robot.x_est);
    x_true(t,:) = robot.x;
    x_est(t,:) = robot.x_est;

end


plot(x_true(:,1),x_true(:,2),'-r')
hold on
plot(x_est(:,1),x_est(:,2),'-b')



