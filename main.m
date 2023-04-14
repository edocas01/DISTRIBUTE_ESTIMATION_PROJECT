% Syntax Functions
% ROBOT(x, y, comradius, id, type) 	type can be 'linear'


clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

% Load config
config;
N = 4;
if N > parameters_simulation.N_MAX
    N = parameters_simulation.N_MAX;
end


%% Test robot
% robot needs x0,y0,comradius,id,type
R = cell(1, N);
target = TARGET(0,0);
for i = 1:N
	R{i} = ROBOT(target.x(1) + 5 * cos(2 * pi / N * (i - 1)), target.x(2) + 5 * sin(2 * pi / N * (i - 1)), 1, i, 'linear');
end



time = 0:parameters_simulation.dt:parameters_simulation.tmax;
x_true = zeros(2, length(time), N);
x_est = zeros(2, length(time), N);
err = zeros(1, length(time), N);

figure(1), clf;
hold on
target.plot();
for i = 1:N
	R{i}.plot();
end
hold off

for t = 1:length(time)-1
	figure(2)
	clf;
	hold on
	target.plot();
	u = [cosd(t); sind(t)];
	for i = 1:N
		EKF(R{i}, u);
		x_true(:,t,i) = R{i}.x;
		x_est(:,t,i) = R{i}.x_est;
		err(:,t,i) = norm(R{i}.x - R{i}.x_est);
		plot(x_true(1, 1:t, i),x_true(2, 1:t, i),'-k')
		plot(x_est(1, 1:t, i),x_est(2, 1:t, i),'-b')
		R{i}.plot();
	end
	drawnow
end
hold off
