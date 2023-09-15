% This script is used to generate the figure of voronoi tessellation

clc;
clear;
close all;
clearvars;
rng default;
addpath('Scripts');
addpath('Functions');
addpath('Classes');

% import parameters
config;
parameters_simulation.N = 2;
N = parameters_simulation.N;
R = cell(N,1);
range = 3;

% define target
T = TARGET([0; 1]);

% define robots
R{1} = ROBOT([-2;-1.5], 1, 'linear' ,parameters_simulation);
R{2} = ROBOT([0.8;-1.1], 2, 'linear' ,parameters_simulation);


% exchange information
for i = 1:15
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
[x,y] = Circle(R{2}.x_est(1), R{2}.x_est(2), R{2}.ComRadius);
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
xlim([-range, range]);
ylim([-range, range]);
xlabel('x [m]');
ylabel('y [m]');
title('Point of view of robot 1');



%{

 
  __     _____  ____   ___  _   _  ___ ___ 
  \ \   / / _ \|  _ \ / _ \| \ | |/ _ \_ _|
   \ \ / / | | | |_) | | | |  \| | | | | | 
    \ V /| |_| |  _ <| |_| | |\  | |_| | | 
     \_/  \___/|_| \_\\___/|_| \_|\___/___|
                                           
 

%}


% plot the voronoi tessellation
figure(2);
clf
hold on;
grid on;
axis equal;

T.plot();

% plot the robots
voronoi_map_consensous(parameters_simulation, R, []);
R{1}.plot_est(all_markers, color_matrix, false);
plot(R{1}.voronoi, 'FaceAlpha', 0.2, 'FaceColor', [1,0,0.8], 'DisplayName', 'Voronoi 1');
[x,y] = Circle(R{1}.x_est(1), R{1}.x_est(2), R{1}.ComRadius/2);
plot(x,y, '--b', 'DisplayName', 'Half com. radius 1', 'LineWidth', 1);

R{2}.plot_est(all_markers, color_matrix, false);
plot(R{2}.voronoi, 'FaceAlpha', 0.2, 'FaceColor', [0.9290 1 0.1250], 'DisplayName', 'Voronoi 2');

[x,y] = Circle(R{2}.x_est(1), R{2}.x_est(2), R{2}.ComRadius/2);
plot(x,y, '--','Color',[0.4660 0.6740 0.1880], 'DisplayName', 'Half com. radius 2', 'LineWidth', 1.5);

% compute voronoi without the volume and uncertainty
R{1}.volume = 0;
R{2}.volume = 0;
R{1}.all_cov_pos = zeros(2*N+2);
R{2}.all_cov_pos = zeros(2*N+2);
R{1}.P = zeros(2);
R{2}.P = zeros(2);
voronoi_map_consensous(parameters_simulation, R, []);

plot(R{1}.voronoi, 'FaceAlpha', 0.2, 'FaceColor', [0.5,0,0.0],  'DisplayName','Voronoi 1 without uncertainty/volume');
plot(R{2}.voronoi, 'FaceAlpha', 0.2, 'FaceColor', [0.7290 0.6940 0.],  'DisplayName','Voronoi 2 without uncertainty/volume');

title('Voronoi tessellation')
xlabel('x [m]');
ylabel('y [m]');

legend('Location','eastoutside')