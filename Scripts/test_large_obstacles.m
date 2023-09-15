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
			if isempty(intersect(LO.poly, [0,0; LO.x(i,:)]))
				visible_points = [visible_points; LO.x(i,:)];
			end
		end
	end

	% if there are no visible vertices of the obstacle inside the cell
	if isempty(visible_points)
		intersection = [];
		index = 0;
		% find visible intersection:
		% find the intersection between the obstacle and the voronoi cell
		LO_vertices = LO.poly.Vertices;
		N_vertices = size(LO_vertices,1);
		% add the first vertex at the end to close the polygon
		LO_vertices = LO_vertices([1:N_vertices,1],:);
		for i = 1:N_vertices
			intersection = linexlines2D(poly_voronoi, LO_vertices(i,:), LO_vertices(i+1,:));
            plot(intersection(1,:),intersection(2,:),'og','MarkerFaceColor','g');
			% if the are intersections between the obstacle and the voronoi cell the delete the area behind
			if (~isempty(intersection))
				index = index + 1;
                points_to_delete = [];
				% define an area to delete
				for j = 1:size(intersection,1)
					dir = intersection(:,j) - [0;0];
					dir = dir/norm(dir);
					% create a point 10 meters behind the intersection
					points_to_delete = [points_to_delete, [intersection(:,j) + 10*dir , intersection(:,j)]];
                end
                plot(points_to_delete(1,:),points_to_delete(2,:),'ok');
				% reorder the points to delete and create a polyshape
                points_to_delete = points_to_delete(:,convhull(points_to_delete'));
				region_to_delete{index} = polyshape(points_to_delete(1,:),points_to_delete(2,:));
				plot(region_to_delete{index})
			end
		end
		% delete the area behind the intersection
		for i = 1:length(region_to_delete)
			poly_voronoi = subtract(poly_voronoi, region_to_delete{i});
		end
		clf;
		hold on;
		grid on;
		axis equal;
		axis(10 * [-1 1 -1 1]);
		plot(poly_voronoi)
	else

	end
end

% 
% pgon=polyshape(     [2.8259    1.8997
%     1.3496    4.4206
%     3.3552    3.4178
%     4.3719    4.6992
%     4.8872    2.1226
%     3.2298    2.7214
%     3.3134    1.1337
%     1.1685    1.1616] );  %hypothetical polygon
% R=1.5; [x0,y0]=deal(3,3); %Circle radius and center
% t=linspace(0,360,1000).'; t(end)=[]; %circle angular samples
% circle=polyshape([cosd(t), sind(t)]*R+[x0,y0]); 
% plot([pgon,circle]); axis equal
% %find intersections
% V=pgon.Vertices;
% N=size(V,1);
% V=V([1:N,1],:);
% hold on
% for i=1:N
%     xy=linexlines2D(circle,V(i,:),V(i+1,:));
%     plot(xy(1,:),xy(2,:),'or','MarkerFaceColor','r');
% end
% hold off
