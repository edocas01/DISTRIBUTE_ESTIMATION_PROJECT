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


%% Assign trajectory
% robot needs x0,y0,comradius,id,type
robot = ROBOT(0,0,0.1,1,'linear');
[target, u_target, obstalces] = initialize_env(parameters_simulation);

% time = 0:parameters_simulation.dt:parameters_simulation.tmax;
time = 0 : parameters_simulation.dt : length(u_target);
x_true = zeros(2, length(u_target));
x_est = zeros(2, length(u_target));
traj = zeros(2, length(u_target));

for t = 1:length(u_target)
    target.dynamics(u_target(:,t));
	EKF(robot, u_target(:,t));

	% err(t,:) = norm(robot.x - robot.x_est);
    x_true(:,t) = robot.x;
    x_est(:,t) = robot.x_est;
    traj(:,t) = target.x;


end

figure(2); clf
plot(x_true(1,:),x_true(2,:),'-k')
hold on
plot(x_est(1,:),x_est(2,:),'-b')
hold on
plot(traj(1,:),traj(2,:),'-r')
for i = 1:length(obstalces)
    plot(obstalces{i}.x(1),obstalces{i}.x(2),'sk')
end



