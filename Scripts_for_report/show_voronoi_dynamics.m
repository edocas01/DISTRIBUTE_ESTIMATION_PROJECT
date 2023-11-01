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
relative_general_consensous(R,T,[],parameters_simulation);
R{1}.all_robots_pos = R{1}.all_robots_pos(1:end-2);
R{2}.all_robots_pos = R{2}.all_robots_pos(1:end-2);
R{1}.all_cov_pos = R{1}.all_cov_pos(1:end-2,1:end-2);
R{2}.all_cov_pos = R{2}.all_cov_pos(1:end-2,1:end-2);
R{1}.ComRadius = 5;
R{2}.ComRadius = 5;
R{1}.vmax = 1000;
R{2}.vmax = 1000;

volume1 = R{1}.volume;
P = R{1}.P;

% define an obstacle
myVideo = VideoWriter('IMAGES/VORONOI/voronoi.avi');
duration = 10;
myVideo.FrameRate = 6 / duration;  
open(myVideo);
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

xticks([]);
yticks([]);
frame = getframe(gcf)
writeVideo(myVideo, frame);
% Initial condition
title('Voronoi explanation','Interpreter','latex');



% compute voronoi without volume and uncertainty
R{1}.volume = 0;
parameters_simulation.coverage = 0;
[enlarged_LO, points_exit] = voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
voronoi1 = R{1}.voronoi;
% plot default voronoi
plot(R{1}.voronoi,'FaceColor', color_matrix(R{1}.id,:))
title('Default Voronoi cell','Interpreter','latex');
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);

frame = getframe(gcf);
writeVideo(myVideo, frame);
clf; hold on; grid on; axis equal;
xticks([]);
yticks([]);

% compute voronoi with volume
R{1}.volume = volume1;
[enlarged_LO, points_exit] = voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
voronoi2 = R{1}.voronoi;
% plot voronoi after volume
for i = 1:length(R)
	R{i}.plot_est(all_markers, color_matrix, true);
end
% large_obstacles{1}.plot();
obstacles{1}.plot();

plot(voronoi1,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.25);
plot(R{1}.voronoi,'FaceColor', color_matrix(R{1}.id,:))
plot(points_exit(1,:), points_exit(2,:), 'xk', 'Markersize', 10, 'Linewidth', 2);
plot(enlarged_LO,'FaceColor', 'k', 'FaceAlpha', 0.5);
title('Volume reduction','Interpreter','latex');
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);

frame = getframe(gcf);
writeVideo(myVideo, frame);
clf; hold on; grid on; axis equal;
xticks([]);
yticks([]);

% compute voronoi with volume and uncertainty on the others
parameters_simulation.coverage = 3;
R{1}.P = eye(2)*0;
[enlarged_LO, points_exit] = voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
voronoi3 = R{1}.voronoi;
% plot voronoi after volume
for i = 1:length(R)
	R{i}.plot_est(all_markers, color_matrix, true);
end
% large_obstacles{1}.plot();
obstacles{1}.plot();

plot(voronoi1,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.11);
plot(voronoi2,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.25);
plot(R{1}.voronoi,'FaceColor', color_matrix(R{1}.id,:))
plot(points_exit(1,:), points_exit(2,:), 'xk', 'Markersize', 10, 'Linewidth', 2);
plot(enlarged_LO,'FaceColor', 'k', 'FaceAlpha', 0.5);
title('Volume reduction + uncertainty on measurements','Interpreter','latex');
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);

frame = getframe(gcf);
writeVideo(myVideo, frame);
clf; hold on; grid on; axis equal;
xticks([]);
yticks([]);

% compute voronoi with volume and uncertainty on the others
R{1}.P = P;
[enlarged_LO, points_exit] = voronoi_map_consensous(parameters_simulation, R, T, obstacles, large_obstacles);
voronoi4 = R{1}.voronoi;
% plot voronoi after volume
for i = 1:length(R)
	R{i}.plot_est(all_markers, color_matrix, true);
end
% large_obstacles{1}.plot();
obstacles{1}.plot();

plot(voronoi1,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.10);
plot(voronoi2,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.17);
plot(voronoi3,'FaceColor', color_matrix(R{1}.id,:), 'FaceAlpha', 0.25);
plot(R{1}.voronoi,'FaceColor', color_matrix(R{1}.id,:))
plot(points_exit(1,:), points_exit(2,:), 'xk', 'Markersize', 10, 'Linewidth', 2);
plot(enlarged_LO,'FaceColor', 'k', 'FaceAlpha', 0.5);
title('Volume reduction + uncertainty on measurements and self position','Interpreter','latex');
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);
xticks([]);
yticks([]);

frame = getframe(gcf);
writeVideo(myVideo, frame);
clf; hold on; grid on; axis equal;
xticks([]);
yticks([]);

for i = 1:length(R)
	R{i}.plot_est(all_markers, color_matrix, true);
end
large_obstacles{1}.plot();
obstacles{1}.plot();

plot(R{1}.voronoi,'FaceColor', color_matrix(R{1}.id,:))
title('Final Voronoi cell','Interpreter','latex');
xlim(parameters_simulation.size_map * [-1 1] / 10);
ylim(parameters_simulation.size_map * [-1 1] / 10);
xticks([]);
yticks([]);

frame = getframe(gcf);
writeVideo(myVideo, frame);
frame = getframe(gcf);
writeVideo(myVideo, frame);
frame = getframe(gcf);
writeVideo(myVideo, frame);

close(myVideo);
