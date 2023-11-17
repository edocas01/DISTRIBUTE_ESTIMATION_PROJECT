close all; clc;
config;
settings_scripts;

parameters_simulation.N = 1;
coverage = parameters_simulation.coverage;

T = TARGET(10^6 * [1;1]);
R = ROBOT([0;0],1,'linear',parameters_simulation);
R.P = R.P*0.02;
R.volume = 0.1;

myVideo = VideoWriter('IMAGES/VORONOI/Voronoi_LO.avi');
myVideo.FrameRate = 7 / 20;  
open(myVideo);

fig = figure(1); clf; hold on; grid on; axis equal; box on
set(gcf,'Position', get(0, 'Screensize'));
xlim((R.ComRadius + 1) * [-1 1]);
ylim((R.ComRadius + 1) * [-1 1]);
title("Inital scenario", 'Interpreter', 'latex')
set(gca,'xtick',[])
set(gca,'ytick',[])
R.plot_est(all_markers, color_matrix, false);
voronoi_map_consensous(parameters_simulation, {R}, T, [], []);
plot_cell_before = plot(R.voronoi, 'FaceColor', color_matrix(1,:));
% LO = create_LO();
plot_LO = LO{1}.plot();

cell_before = R.voronoi;

[enlarged_LO, ~, LO_measured_vertices, region_to_delete] = voronoi_map_consensous(parameters_simulation, {R}, T, [], LO);
LO_measured_vertices = unique(LO_measured_vertices, 'rows');


frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);

title("Robot point of view", 'Interpreter', 'latex')
plot_vert = plot(LO_measured_vertices(:,1), LO_measured_vertices(:,2), 'or', 'MarkerSize', 10);


frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
title("Area to be deleted", 'Interpreter', 'latex')
delete(plot_LO);
plot_reg_to_delete = plot(region_to_delete, 'FaceColor', 'k', 'FaceAlpha', 0.5);

% obs_regs = obs_regions(R, LO_measured_vertices);

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
delete(plot_cell_before);
delete(plot_vert)
plot_newcell = plot(subtract(cell_before, region_to_delete), 'FaceColor', color_matrix(1,:));

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
title("Obstacle inflation", 'Interpreter', 'latex')
delete(plot_newcell)
delete(plot_reg_to_delete)
plot_enlarge_LO = plot(enlarged_LO, 'FaceColor', 'k', 'FaceAlpha', 0.5);
plot_nonconvexcell = plot(subtract(cell_before, enlarged_LO),  'FaceColor', color_matrix(1,:));

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
title("Managing starred convexity", 'Interpreter', 'latex')
delete(plot_enlarge_LO)
plot(R.voronoi, 'FaceColor', color_matrix(1,:));
plot_LO = LO{1}.plot();

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);
title("Final cell", 'Interpreter', 'latex')
delete(plot_nonconvexcell)

frame = getframe(gcf); %get frame
writeVideo(myVideo, frame);

close(myVideo);
