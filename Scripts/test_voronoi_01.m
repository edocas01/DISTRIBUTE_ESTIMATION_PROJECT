close all;
clc;
clearvars;
addpath('..\Classes\');
addpath('..\Functions\');
addpath('..\');
config;
N = 10;
robots = cell(N,1);
range = 5;
for i = 1:N
	x(i) = (rand() - 0.5)*range;
	y(i) = (rand() - 0.5)*range;
	robots{i} = ROBOT([x(i),y(i)],i,'linear',parameters_simulation);
end



P = [x', y'];
% Find the voroni map
[vx,vy] = voronoi(x,y); % vx, vy are the vertices of the voronoi edges
[V,C] = voronoin(P); % V is the vertices of the voronoi cells, C is the index of the vertices of the voronoi cells

[x_circle,y_circle] = Circle(robots{1}.x(1), robots{1}.x(2), robots{1}.ComRadius, 'k', false);
close all;
circle = polyshape(x_circle,y_circle);

for i = 1:length(C)
	out_index = []; % The index of the points that are outside the circle
	inf_index = []; % The index of the points that are at infinite
	% If the index is 1, then it is an infinite point
	if sum(C{i} == 1)
		inf_index = find(C{i} == 1);
	end
	for j = 1:length(C{i})
		[in on] = inpolygon(V(C{i}(j),1),V(C{i}(j),1),circle.Vertices(:,1),circle.Vertices(:,2));
		% Remove the points that are outside the circle
		if in == 0 && on == 0
			out_index = [out_index, j];
		end
	end
	out_index = [inf_index out_index];
	C{i}(out_index) = [];
end

% Find intersection points between the voronoi edges and the circle
cross_point = [];
for i = 1:length(vx)
	[xi,yi] = polyxpoly(vx(:,i),vy(:,i),circle.Vertices(:,1),circle.Vertices(:,2),'unique');
	cross_point = [cross_point; [xi,yi]];
end

figure()
hold on;
plot(robots{1}.x(1),robots{1}.x(2),'ob');
for i = 2:N
	plot(robots{i}.x(1),robots{i}.x(2),'or');
end
plot(vx,vy,'k');
for i = 1:length(cross_point)
	plot(cross_point(i,1),cross_point(i,2),'*g');
end



old_V = V;
V = [V;cross_point];
% Check the added points
C_pos = P;
for i = length(old_V)+1:length(V)
	V_dist = sum(abs(V(i,:)-C_pos).^2,2).^0.5; % distance between the new point and the robots
	[~,V_index] = min(V_dist);
	if length(V_index)==2
		C{V_index(1)}=[C{V_index(1)} i];
		C{V_index(2)}=[C{V_index(2)} i];
	else
		C{V_index}=[C{V_index} i];
		V_dist(V_index) = max(V_dist); 
		[~,V_index] = min(V_dist);
		C{V_index}=[C{V_index} i];
	end
end


plot(V(C{1},1),V(C{1},2),'*b');
