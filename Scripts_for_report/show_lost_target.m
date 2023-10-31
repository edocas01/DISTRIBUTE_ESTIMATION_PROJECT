clc;
set(0,'defaultAxesFontSize', 11)
set(0,'DefaultLegendFontSize',11)

myVideo = VideoWriter('IMAGES/SIMULATION_VIDEO/video.avi');
duration = 45; % sec
myVideo.FrameRate = length(results)/duration; 
open(myVideo);

fig = figure(1);
set(gcf, 'Position', get(0, 'Screensize'));
get(0,'defaultfigureposition');
norm_P = zeros(length(results), length(results{1}.R));

for t = 1:length(results)
	clf;
	time = 0:1:t-1;
	
	subplot(2,1,1); 
	show_simulation({results{t}});
	
	subplot(2,1,2); hold on; grid on; box on;
	for i = 1:length(results{t}.R)
		norm_P(t,i) = norm(results{t}.R{i}.all_cov_pos(end-1:end,end-1:end));
		plot(time, norm_P(1:t,i),'Color', color_matrix(i,:), 'DisplayName', ['R. ' num2str(i)]);
	end
	title('Covariance on the target estimates', 'Interpreter', 'latex'); 
	ylabel('[$m^2$]', 'Interpreter', 'latex');
	xlabel('Time [s]', 'Interpreter', 'latex');

	xlim([0, t]);
	legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');


	drawnow
	
	frame = getframe(gcf); %get frame
	writeVideo(myVideo, frame);
end
close(myVideo)


