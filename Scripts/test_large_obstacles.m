clc;
close all;
clearvars;
rng default;
config;
parameters_simulation.N = 1;
% create the robot
radius_voronoi = 5;

R = ROBOT([2.78;1.45], 1, 'linear', parameters_simulation);
for i = 1:10
	EKF(R,0);
end
[cx,cy] = Circle(R.x_est(1),R.x_est(1),radius_voronoi);
poly_voronoi = polyshape(cx,cy);
R.voronoi = poly_voronoi;


% create the large obstacles
fig_1 = figure(1); clf;
% Set the large obstacles
sgtitle("Select points to create obstacles")
hold on;
grid on;
axis equal;
axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);

R.plot_real(all_markers, color_matrix, true);
plot(R.voronoi);
idx = 1;
FIRST = false;
while true
	x = [];
	y = [];
	X = [];
	while true
		if ~FIRST
			[xi, yi ,button] = ginput(1);
		end
		if size(X,1) > 2
			if ~isequal(button,1) % if enter is pressed
				break;
			end
		end
		FIRST = false;
		x = [x xi];
		y = [y yi];
		X = [x' y'];
		plot(x, y, '--ko');
	end
	large_obstacles{idx} = LARGE_OBSTACLE(X);
	clf;
	sgtitle("Select points to create obstacles")
	hold on;
	grid on;
	axis equal;
	axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);
	for i = 1:length(large_obstacles)
		large_obstacles{i}.plot();
	end
	R.plot_real(all_markers, color_matrix, true);
	plot(R.x_est(1), R.x_est(2), 'kx');
	plot(R.voronoi);
	idx = idx + 1;
	
	[xi,yi, button] = ginput(1);
	if ~isequal(button,1) % if enter is pressed
		break;
	else
		FIRST = true;
	end
end


[~, eigenvalues] = eig(R.P*3);
max_semiaxis = sqrt(max(diag(eigenvalues)));

% reduct the cell according to large obstacles
for i = 1:length(large_obstacles)
	voronoi_LO(large_obstacles{i}, R, max_semiaxis, parameters_simulation);
end

figure(2);
hold on;
grid on;
axis equal;
axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);
for i = 1:length(large_obstacles)
	large_obstacles{i}.plot();
end
R.plot_real(all_markers, color_matrix, true);
plot(R.voronoi);