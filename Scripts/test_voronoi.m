close all;
clc;
clearvars;

points = 10;
range = 5;
for i = 1:points
	x(i) = (rand() - 0.5)*range;
	y(i) = (rand() - 0.5)*range;
end

% Plot the points
plot(x, y, 'or');
hold on
P = [x', y'];
% Find the voroni map
[v,c] = voronoin(P);

% Plot the voroni map for the a generic point
idx = round(rand()*points);
for idx = 1:points
for i = 1:length(c{idx})
	plot(v(c{idx}(i),1), v(c{idx}(i),2), 'b*');
end
end

figure()
[a,b] = voronoi(x,y);
plot(a,b,'b')
hold on 
plot(x, y, 'or');



dt = delaunayTriangulation(P);
[V,R] = voronoiDiagram(dt);

