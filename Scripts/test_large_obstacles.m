clc;
close all;
clearvars;
rng default;
config;

radius_voronoi = 5;
[cx,cy] = Circle(0,0,radius_voronoi);
poly_voronoi = polyshape(cx,cy);


x = [];
y = [];
X = [];
fig_1 = figure(1); clf;
% Set the large obstacles
sgtitle("Select points to create obstacles")
hold on;
grid on;
axis equal;
axis(10 * [-1 1 -1 1]);

plot(poly_voronoi)

while true
	
	[xi, yi ,button] = ginput(1);
	
	if size(X,1) > 2
		if ~isequal(button,1) % if enter is pressed
			break;
		end
	end
	x = [x xi];
	y = [y yi];
	X = [x' y'];
	plot(x, y, '--ko');
    axis(10 * [-1 1 -1 1]);
end
LO = LARGE_OBSTACLE(X);
clf;
sgtitle("Select points to create obstacles")
hold on;
grid on;
axis equal;
axis(10 * [-1 1 -1 1]);
LO.plot();
plot(poly_voronoi)
plot(0,0,'*k');



% check if the obstacle is inside the voronoi cell
visible_points = [];
intersection = intersect(LO.poly, poly_voronoi);
if ~isempty(intersection) % there is intersection
	% check if the are vertices of the obstacle inside the cell
	for i = 1 : size(LO.x,1)
		% if there are vertices of the obstacle inside the cell
		if inpolygon(LO.x(i,1), LO.x(i,2), poly_voronoi.Vertices(:,1), poly_voronoi.Vertices(:,2))
			% if I can see this point from the origin I use this point (the line connecting me and the point is inside the obstacle or not)
			if ~isempty(intersect(LO.poly, [0,0; LO.x(i,:)]))
				visible_points = [visible_points; LO.x(i,:)];
			end
		end
	end

	% if there are no visible vertices of the obstacle inside the cell
	if isempty(visible_points)
		% 
	else

	end
end
