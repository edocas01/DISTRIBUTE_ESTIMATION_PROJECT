clc;
set(0,'defaultAxesFontSize', 11)
set(0,'DefaultLegendFontSize',11)
% extract metrics from results
metrics = compute_metrics(results, parameters_simulation);

% myVideo = VideoWriter('IMAGES/SIMULATION_VIDEO/video.avi');
% duration = 45; % sec
% myVideo.FrameRate = length(results)/duration; 
% open(myVideo);
% 
% fig = figure(1);
% set(gcf, 'Position', get(0, 'Screensize'));
% get(0,'defaultfigureposition');
% for t = 1:length(results)
% 	clf;
% 	time = 0:1:t-1;
% 	
% 	subplot(2,2,1); hold on; grid on; box on;
% 	for i = 1:length(metrics)
% 		if results{t}.R{i}.type == "unicycle"
% 			dyn = ' NL';
% 		else
% 			dyn = ' L';
% 		end
% 		tmp = metrics{i}.err_dist(1:t);
% 		tmp(tmp > 3*parameters_simulation.DISTANCE_TARGET) = NaN;
% 		plot(time, tmp,'Color', color_matrix(i,:), 'DisplayName', ['R. ' num2str(i) dyn]);
% 	end
% 	title('Distance on target error', 'Interpreter', 'latex'); 
% 	ylabel('Error [m]', 'Interpreter', 'latex');
% 	set(gca,'xtick',[])
% 
% 	xlim([0, length(metrics{1}.err_dist(1:t))]);
% % 	legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
% 
% 	subplot(2,2,3); hold on; grid on; box on;
% 	box on;
% 	for i = 1:length(metrics)
% 		if results{t}.R{i}.type == "unicycle"
% 			dyn = ' NL';
% 		else
% 			dyn = ' L';
% 		end
% 		tmp = metrics{i}.err_angles(1:t);
% 		tmp(tmp == 100) = NaN;
% 		plot(time, tmp, 'Color', color_matrix(i,:), 'DisplayName', ['R. ' num2str(i) dyn]);
% 	end
% 	title('Equidistance angle error', 'Interpreter', 'latex'); 
% 	ylabel('Error [rad]', 'Interpreter', 'latex');
% 	xlabel('Time [s]', 'Interpreter', 'latex');
% 	ylim("padded");
% 	xlim([0, length(metrics{1}.err_dist(1:t))]);
% % 	legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
% 
% 	subplot(2,2,[2 4]); 
% 	show_simulation({results{t}});
% 	drawnow
% 	
% 	frame = getframe(gcf); %get frame
% 	writeVideo(myVideo, frame);
% end
% close(myVideo)
values = [];
% prepare a table of metrices for the presentation
for j = 1:length(metrics)
	err_dist = metrics{j}.err_dist;
	err_angle = metrics{j}.err_angles;
	mean_err_dist = mean(err_dist(err_dist < 3*parameters_simulation.DISTANCE_TARGET));
	std_err_dist = std(err_dist(err_dist < 3*parameters_simulation.DISTANCE_TARGET));
	mean_err_angle = mean(err_angle(err_angle < 100));
	std_err_angle = std(err_angle(err_angle < 100));
	values(1,j) = mean_err_dist;
	values(2,j) = std_err_dist;
	values(3,j) = mean_err_angle;
	values(4,j) = std_err_angle;
end

