clc;
set(0,'defaultAxesFontSize', 11)
set(0,'DefaultLegendFontSize',11)
% extract metrics from results

myVideo = VideoWriter('IMAGES/DIFFERENT_Rc/video.avi');
duration = 45; % sec
myVideo.FrameRate = length(results)/duration; 
open(myVideo);

fig = figure(1);
set(gcf, 'Position', get(0, 'Screensize'));
get(0,'defaultfigureposition');
for t = 1:length(results)
	clf;
	show_simulation({results{t}});
	drawnow
	frame = getframe(gcf); %get frame
	writeVideo(myVideo, frame);
end
close(myVideo)


