% This script is used to generate the figure of voronoi tessellation

clc;
clear;
close all;
clearvars;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

% import parameters
config;
N = parameters_simulation.N;
R = cell(N,1);
range = 3;

% define target
T = TARGET([0; 0]);

% define robots
for i = 1:N
	R{i} = ROBOT([randi([-range, range]); randi([-range, range])], i, 'linear' ,parameters_simulation);
end

% exchange information
for i = 1:10
	for j = 1:length(R)
		EKF(R{j},0);
	end
	relative_general_consensous(R, T, parameters_simulation);
end

% plot the robot and the target
figure(1); 
clf
hold on;
grid on; 
axis equal;

% plot the target
T.plot();

% plot the robots
R{1}.plot_est(all_markers, color_matrix, false);
[x,y] = Circle(R{1}.x_est(1), R{1}.x_est(2), R{1}.ComRadius);
plot(x,y, '--', 'Color', color_matrix(R{1}.id,:), 'DisplayName', 'Com. radius 1', 'LineWidth', 1);

plot(R{2}.x_est(1), R{2}.x_est(2), 'ok', 'DisplayName', 'Other robot','MarkerSize', 10,'LineWidth', 0.8);
[x,y] = Circle(R{2}.x_est(1), R{2}.x_est(2), R{1}.ComRadius);
plot(x,y, '--k', 'DisplayName', 'Other com. radius', 'LineWidth', 1.5);

% plot the covariance ellipses of robot 1
clear x y
for i = 1:N+1
	x = R{1}.all_robots_pos(2*(i-1)+1);
	y = R{1}.all_robots_pos(2*(i-1)+2);
	COV = R{1}.all_cov_pos(2*(i-1)+1:2*(i-1)+2, 2*(i-1)+1:2*(i-1)+2);
	point_ellipse = compute_ellipse([x,y], COV, 3);
	point_ellipse = polyshape(point_ellipse(1,:), point_ellipse(2,:));
	if i < N+1
		plot(point_ellipse, 'HandleVisibility', 'off', 'FaceAlpha', 0.2, 'FaceColor', [0.1,0.5,0.1]);
	else
		plot(point_ellipse, 'DisplayName', 'Cov. ellipses computed from 1', 'FaceAlpha', 0.2, 'FaceColor', [0.1,0.5,0.1]);
	end
end


legend;
hold off
% xlim([-range range]); ylim([-range range]);
% pause(1)
% %% Calculations	
% tic
% results = run_simulation(R, T, u_traj, parameters_simulation);
% toc

% %% Animation
% tic
% figure(2);

% for t = 1:length(results)
%     clf
% 	hold on; grid on; 
% 	xlim([-40 40]); ylim([-40 40]);
% 	datas = results{t};
% 	datas.T.plot()
% 	plot(datas.circle_target(1,:), datas.circle_target(2,:), 'b--', 'LineWidth', 1.5);
% 	for i = 1:N
% 		datas.R{i}.plot_real(all_markers, color_matrix, false);
% 		plot(datas.R{i}.voronoi);
% 		plot(datas.barycenter(1,i), datas.barycenter(2,i), 'kx', 'LineWidth', 1);
%     end
%     drawnow
% end

% toc