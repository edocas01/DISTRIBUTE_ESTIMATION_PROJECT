% clc; clear; close all;
clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');

coverage = 3;
config;
N = 8;
range = 15;

dyn_type = repmat("linear",N,1);
T = TARGET([0;0]);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

figure(1)
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot(all_markers, color_matrix, false);
end
hold off
for i = 1:N
	for j = 1:10
    	EKF(R{i}, 0)
	end
end


relative_target_consensous(R, T, parameters_simulation);
voronoi_map(parameters_simulation, R, [], coverage);

% phi = @(x,y) exp(-((x - T.x(1))^2 + (y - T.x(2))^2)); % kg / m^2
func = @(x,y,r,x_t,y_t) exp(-r/200*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2);
R_form = 10;
phi = @(x,y) func(x, y, R_form, T.x(1), T.x(2));

ptsx = zeros(N+1, 1);
ptsy = zeros(N+1, 1);
for i = 1:N
	ptsx = [ptsx; R{i}.x_est(1)];
	ptsy = [ptsy; R{i}.x_est(2)];
end

[vx, vy] = voronoi(ptsx, ptsy);

tic
[circx, circy] = circle(T.x(1), T.x(2), R_form);

figure(2)
T.plot();
hold on; grid on; axis equal;
plot(vx, vy, 'k', 'HandleVisibility', 'off')


h = zeros(1,N+1);
for i = 1:N
	[barycenter, msh] = compute_centroid(R{i}.voronoi, phi);
	h(i) = R{i}.plot(all_markers, color_matrix, false);
	
	if i == N
	 	h(i+1) = plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off','DisplayName','Centroid');
	else
		plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off')
	end
	plot(R{i}.voronoi, 'HandleVisibility', 'off')
	pdemesh(msh);
end
legend(h, 'Location', 'bestoutside')
xlim([-30 30])
ylim([-30 30])
hold off

toc


