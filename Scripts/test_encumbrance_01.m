clc;
close all;

N = 5;
range = 5;
volume = 0.5;

figure()
hold on
axis equal
for i = 1:N
	agents{i}.x = [(rand()- 0.5)*range;(rand()- 0.5)*range];
	agents{i}.volume = rand()*volume;
	[agents{i}.xcircle, agents{i}.ycircle] = Circle(agents{i}.x(1),agents{i}.x(2),agents{i}.volume);
	plot(agents{i}.x(1),agents{i}.x(2),'ok')
	plot(agents{i}.xcircle,agents{i}.ycircle,'-');
	P(i,:) = agents{i}.x;
end

voronoi(P(:,1),P(:,2))