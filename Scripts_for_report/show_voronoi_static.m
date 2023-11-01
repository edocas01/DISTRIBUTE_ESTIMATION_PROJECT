clear; close all; clc;
config;
parameters_simulation.N = 2;
coverage = parameters_simulation.coverage;
% initialize 2 robots
R = {};
R{1} = ROBOT([0;0],1,'linear',parameters_simulation);
R{2} = ROBOT([3;2],2,'linear',parameters_simulation);
T = TARGET([1;1]*1e5);
for i = 1:10
	for j = 1:length(R)
		EKF(R{j},[0;0])
	end
end
relative_general_consensous(R,T,parameters_simulation);
R{1}.all_robots_pos = R{1}.all_robots_pos(1:end-2);
R{2}.all_robots_pos = R{2}.all_robots_pos(1:end-2);
R{1}.all_cov_pos = R{1}.all_cov_pos(1:end-2,1:end-2);
R{2}.all_cov_pos = R{2}.all_cov_pos(1:end-2,1:end-2);
R{1}.ComRadius = 5;
R{2}.ComRadius = 5;
R{1}.vmax = 1000;
R{2}.vmax = 1000;
volume1 = R{1}.volume;
volume2 = R{2}.volume;

% define an obstacle
fig = figure(1); clf; hold on; grid on; axis equal;
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);
for i = 1:length(R)
	R{i}.plot_est(all_markers, color_matrix, true);
end
idx = 1;
FIRST = false;
first_time = true;
exit = false;
while true
	x = [];
	y = [];
	X = [];
	while true
		if ~FIRST
			[xi, yi ,button] = ginput(1);
		end

		if first_time
			if ~isequal(button,1) % if enter is pressed
				exit = true;
				break;
			end
		end
		first_time = false;

		if size(X,1) > 2
			if ~isequal(button,1) % if enter is pressed
				break;
			end
		end
		FIRST = false;
		x = [x xi];
		y = [y yi];
		X = [x' y'];
		plot(x, y, '--k','HandleVisibility','off');
	end 
	
	if exit
		break;
	end
	large_obstacles{idx} = LARGE_OBSTACLE(X);
	
	for i = 1:length(large_obstacles)
		large_obstacles{i}.plot();
	end
	break;
end

% create a small obstacle
idx = 1;
x = [];
y = [];
while true
	[xi, yi ,button] = ginput(1);
	x = [x xi];
	y = [y yi];
	if ~isequal(button,1) % if enter is pressed
		break;
	end

	obstacles{idx} = OBSTACLE(xi,yi, true, parameters_simulation);
	plot(x, y, 'sb','Linewidth', 2, 'Markersize', 10,'DisplayName', 'Moving obstacle');
	% obstacles{idx}.plot();
	break;
end


% compute voronoi without volume and uncertainty
R{1}.volume = 0;
R{2}.volume = 0;
parameters_simulation.coverage = 0;
voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
% plot the regions of voronoi
style = "-";
R{1}.plot_voronoi_edge(style,color_matrix,'Default cell');
R{2}.plot_voronoi_edge(style,color_matrix);


% compute voronoi with volume
R{1}.volume = volume1;
R{2}.volume = volume2;
voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
style = "--";
R{1}.plot_voronoi_edge(style,color_matrix,'Volume reduction');
R{2}.plot_voronoi_edge(style,color_matrix);
% compute voronoi with volume and uncertainty
parameters_simulation.coverage = 3;
voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
plot(R{1}.voronoi,'Facecolor', color_matrix(1,:), 'FaceAlpha', 0.2, 'DisplayName', 'Full reduction');
plot(R{2}.voronoi,'Facecolor', color_matrix(2,:), 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot([R{1}.x_est(1),R{1}.x_est(1)+R{1}.ComRadius*cos(170*pi/180)], [R{1}.x_est(2),R{1}.x_est(2)+R{1}.ComRadius*sin(170*pi/180)], '-.k', 'HandleVisibility', 'off');
% add a text on the radius
text(R{1}.x_est(1)+R{1}.ComRadius*0.8*cos(170*pi/180),R{1}.x_est(2)+R{1}.ComRadius*1.1*sin(170*pi/180), '$R_{com}$', 'Interpreter', 'latex', 'FontSize', 20);

box on;
xticks([]);
yticks([]);
settings_scripts;
title('Voronoi cell construction','Interpreter','latex');
legend('Location','southwest','Interpreter','latex', 'NumColumns', 2);
set(gcf, 'Position', get(0, 'Screensize'));
saveas(fig, 'IMAGES/VORONOI/voronoi_cell_construction.png');
saveas(fig, 'IMAGES/VORONOI/voronoi_cell_construction.fig');