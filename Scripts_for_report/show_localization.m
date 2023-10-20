clc; clear; close all;

settings_scripts;
config;
parameters_simulation.N = 1;
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

R1 = ROBOT([0;0], i, 'linear', parameters_simulation);
R1.ComRadius = 1e5;

R = {R1};
Tx = [T.x(1)]; Ty = [T.x(2)];
for i = 1:length(u_trajectory(1,:));
	T.dynamics([i, sin(i)]);
	relative_general_consensous(R, T, parameters_simulation);
	R{1}.target_est_hist(end+1,:) = [R{1}.all_robots_pos(end-1), R{1}.all_robots_pos(end)];
end

figure(1); grid on; hold on;
plot(u_trajectory(1,:), 'DisplayName', 'x');	
plot(R{1}.target_est_hist(:,1), '--r', 'DisplayName', 'Target estimated trajectory');
legend('Location', 'best');


