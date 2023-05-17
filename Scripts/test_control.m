clc;
close all;
clearvars;
% rng default;
addpath('Classes');
addpath('Functions');

config;
% [T, trajectory, u_trajectory, obstacles] = initialize_env(parameters_simulation);
T = TARGET([0;0]);
fprintf("Target initial position: (%.2f m, %.2f m)\n", T.x(1), T.x(2));

coverage = 3;
N = 10;
range = 15;

dyn_type = repmat("linear",N,1);
R = select_shape(N, dyn_type, 'circle', T.x, range, 0, parameters_simulation);

figure(1); clf
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot(all_markers, color_matrix, false);
end
hold off

%% 
for i = 1:N
	for j = 1:10
    	EKF(R{i}, 0)
	end
end

relative_target_consensous(R, T, parameters_simulation);
voronoi_map(parameters_simulation, R, [], coverage);

func = @(x,y,r,x_t,y_t) exp(-r/200*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2);
R_form = 10;
phi = @(x,y) func(x, y, R_form, T.x(1), T.x(2));

[circx, circy] = Circle(T.x(1), T.x(2), R_form);

figure(2); clf
T.plot();
hold on; grid on; axis equal;
h = zeros(1,N+1);
plot(circx, circy, '--', 'HandleVisibility','off')
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



%% Animation
tic
[circx, circy] = Circle(T.x(1), T.x(2), R_form);
x_est_hist = cell(N,1);
barycenter_hist = cell(N,1);

Tmax = 10;
kp = 1 / parameters_simulation.dt;

for t = 1:parameters_simulation.dt:Tmax
	figure(2); clf
	xlim([-30 30])
	ylim([-30 30])
	hold on; grid on; axis equal;
    plot(circx, circy, '--', 'HandleVisibility','off')
	h = zeros(1,N+1);
	for i = 1:N
        relative_target_consensous(R, T, parameters_simulation);
		title(sprintf("Time: %.2f s", t))
        voronoi_map(parameters_simulation, R, [], coverage);
        
		[barycenter, msh] = compute_centroid(R{i}.voronoi, phi);

		if  kp * norm(barycenter - R{i}.x_est) < R{i}.vmax
			u = kp * (barycenter - R{i}.x_est) * parameters_simulation.dt;
		else
			u = R{i}.vmax * parameters_simulation.dt * (barycenter - R{i}.x_est) / norm(barycenter - R{i}.x_est);
		end
		
		EKF(R{i}, u);
		
		
		h(i) = R{i}.plot(all_markers, color_matrix, false);
		plot(R{i}.voronoi, 'HandleVisibility', 'off')
		if i == N
			h(i+1) = plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off','DisplayName','Centroid');
		else
			plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off')
		end
	end
	T.plot();
	legend(h, 'Location', 'bestoutside')
	pause(0.001)
end
hold off



% for i = 1:N
	
% 	pdemesh(msh);
% end


toc