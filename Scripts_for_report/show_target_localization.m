clc; clear; close all;

settings_scripts;
config;
dt = parameters_simulation.dt;
r = 3;
fig_1 = figure(1); clf; hold on; grid on; axis equal;
axis(parameters_simulation.size_map * [-1 1 -1 1]);
sgtitle("Select points to create trajectory")
% Set the target trajectory
x = [];
y = [];
while true
	[xi, yi ,button] = ginput(1);
	x = [x xi];
	y = [y yi];
	if ~isequal(button,1) % if enter is pressed
		break;
	end
	plot(x, y, '--ro');
end
close(fig_1);
trajectory = [x;y];
n = length(x);
u_trajectory = [];
% inputs for the target
for i = 1:n-1
	% define the initial and final points
	x_in = [x(i); y(i)];
	x_fin = [x(i+1); y(i+1)];
	% compute distance from the point to the next point
	dist = norm(x_fin - x_in);
	% compute the number of steps as the distance divided by the time step
	N_steps = round(dist / (dt*parameters_simulation.vmax_target));
	% compute the velocity
	for j = 1:N_steps
		% the iput is equal to the velocity for a certain number of steps
		u_trajectory = [u_trajectory, (x_fin - x_in)/N_steps];
	end
end
	
% generate the target
T = TARGET([x(1),y(1)]);
% define the number of robot with which perform the localization of the target
number_robots = [1:1:15];
R = cell(1, size(number_robots,2)); % {{R1}, {R1, R2, R3, R4, R5}, {R1, R2, R3, R4, R5, R6, R7}}
for i = 1:length(R)
	parameters_simulation.N = number_robots(i);

	tmp =  cell(1, number_robots(i));

	for j = 1:number_robots(i)
		tmp{j} = ROBOT([rand();rand()], j, 'linear', parameters_simulation);
		tmp{j}.R_dist = eye(2)*(parameters_simulation.std_relative_sensor^2);
		tmp{j}.ComRadius = 1e5;
	end
	R{i} = tmp;

end


Tx = [T.x(1)]; Ty = [T.x(2)];
Tx = []; Ty = [];
for i = 1:length(u_trajectory(1,:))
	
	T.dynamics(u_trajectory(:,i));
	Tx = [Tx, T.x(1)]; Ty = [Ty, T.x(2)];
	
	for j = 1:size(number_robots,2)
		relative_general_consensous(R{j}, T, parameters_simulation);
		R{j}{1}.target_est_hist(end+1,:) = [R{j}{1}.all_robots_pos(end-1), R{j}{1}.all_robots_pos(end)];
	end
end
% crate a matrix to store the distance errors computed with the differrent groups of robots (Nx3)
err = zeros(length(u_trajectory(1,:)), length(number_robots));
for i = 1:length(number_robots)
	err(:,i) = sum((R{i}{1}.target_est_hist - [Tx',Ty']).^2, 2).^0.5;
end

% Plot errors
fig = figure(1); grid on; hold on;
set(gcf, 'Position', get(0, 'Screensize'));
color = ['k','r','b'];
idx = 1;
for i = 1:length(number_robots)
	if number_robots(i) == 1 || number_robots(i) == 7 || number_robots(i) == 15
		plot(0:(size(err,1)-1),err(:,i), 'LineWidth', 1, 'DisplayName', ['Number of robots = ', num2str(number_robots(i))], 'Color', color(idx));
		idx = idx + 1;
	end
end
title("Target estimation error vs number of robots");
xlabel('Time [s]');
ylabel('Error [m]');
xlim([0 size(err,1)-1]);
legend('Location', 'northwest');

% save the figure
saveas(fig, 'IMAGES/TARGET_ESTIMATION/err_vs_number_robots.png');
saveas(fig, 'IMAGES/TARGET_ESTIMATION/err_vs_number_robots.fig');

% compute the mean of the errors
mean_err = mean(err,1);
% Plot mean errors
fig = figure(2); grid on; hold on;
set(gcf, 'Position', get(0, 'Screensize'));
stem(mean_err, 'LineWidth', 1);
xticks(1:length(number_robots));
xlim([1 length(number_robots)]);
title("Mean target estimation error vs number of robots");
xlabel('Number of robots');
ylabel('Mean error [m]');

% save the figure
saveas(fig, 'IMAGES/TARGET_ESTIMATION/mean_err_vs_number_robots.png');
saveas(fig, 'IMAGES/TARGET_ESTIMATION/mean_err_vs_number_robots.fig');

for i = 1:length(number_robots)
	% create a latex macro to store the mean error for each number of robots
	create_macro_latex("latex_macros.tex", "mean_err_"+num2str(number_robots(i))+"R", mean_err(i), "a")
end