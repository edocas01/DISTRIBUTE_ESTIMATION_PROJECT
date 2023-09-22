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
count_visible_points = 0;
intersection = intersect(LO.poly, poly_voronoi);
LO_vertices_copy = LO.x; % This matrix has on the third column a 1 if the vertex is visible and 0 otherwise
LO_vertices_copy(:,3) = -1; % -1 means that the vertex is outside the voronoi cell

%{
LO_vertices:
x | y | tag
- | - | -1  -> vertix of the obstacle outside the voronoi cell
- | - | 0   -> vertix of the obstacle not visible from the robot
- | - | 1   -> vertix of the obstacle visible from the robot
%}


if ~isempty(intersection) % there is intersection
	% check if the are vertices of the obstacle inside the cell
	for i = 1 : size(LO.x,1)
		% if there are vertices of the obstacle inside the cell
		if inpolygon(LO.x(i,1), LO.x(i,2), poly_voronoi.Vertices(:,1), poly_voronoi.Vertices(:,2))
			% if I can see this point from the origin I use this point (the line connecting me and the point is inside the obstacle or not)
			if isempty(intersect(LO.poly, [0,0; LO.x(i,:)]))
				LO_vertices_copy(i,3) = 1;
				count_visible_points = count_visible_points + 1;
			else
				LO_vertices_copy(i,3) = 0;
			end
		end
	end

	% THERE ARE NO VISIBLE VERTICES OF THE OBSTACLE INSIDE THE VORONOI CELL
	if count_visible_points == 0 
		intersection_voronoi = [];
		index = 0;
		% find visible intersection:
		% find the intersection between the obstacle and the voronoi cell
		LO_vertices = LO.poly.Vertices;
		N_vertices = size(LO_vertices,1);
		% add the first vertex at the end to close the polygon
		LO_vertices = LO_vertices([1:N_vertices,1],:);
		for i = 1:N_vertices
			intersection_voronoi = linexlines2D(poly_voronoi, LO_vertices(i,:), LO_vertices(i+1,:)); % output given by column
			% check if the intersection_voronoi is visible:
			% notice that the function linexlines2D returns the intersection between the voronoi cell and the obstacle but the 
			% output points are an approximation of the real ones.
			% So in order to verify if such points are visible or not we remove from the function "intersect" the points that may be 
			% equal the one given by the function linexlines2D
			visible_intersection = [];
			for k = 1:size(intersection_voronoi,2)
				intersection_obstacle = intersect(LO.poly, [0,0;intersection_voronoi(1,k), intersection_voronoi(2,k)]); % output given by row
				distances = sum(abs(intersection_obstacle - [intersection_voronoi(1,k), intersection_voronoi(2,k)]).^2,2).^0.5;
				intersection_obstacle(distances < 1e-4,:) = [];
				% If after removing "itself" there are still intesections with the obstacle then the intersection with voronoi is not visible
				% and then we proced with the reduction of the cell
				if isempty(intersection_obstacle)
					visible_intersection = [visible_intersection, intersection_voronoi(:,k)]; % output given by column
					plot(intersection_voronoi(1,k),intersection_voronoi(2,k),'og','MarkerFaceColor','g');
				end
			end
			% if the are visible intersections between the obstacle and the voronoi cell the delete the area behind
			if (~isempty(visible_intersection))
				if size(visible_intersection,2) ~= 2
					error("Problem with intersections between the obstacle and the voronoi cell");
				end
				index = index + 1;
                points_to_delete = [];

				% TODO: move the intersections accordingly to the uncertainties

				% define an area to delete for the first point
				dir = visible_intersection(:,1) - [0;0];
				dir = dir/norm(dir);
				% create a point 1000 meters behind the intersection
				points_to_delete = [points_to_delete, [visible_intersection(:,1) + 1000*dir , visible_intersection(:,1)]]; % matrix 2 by n
                
				% define an area to delete for the second point
				dir = visible_intersection(:,2) - [0;0];
				dir = dir/norm(dir);
				% create a point 1000 meters behind the intersection
				points_to_delete = [points_to_delete, [visible_intersection(:,2) + 1000*dir , visible_intersection(:,2)]];

				% reorder the points to delete and create a polyshape
                points_to_delete = points_to_delete(:,convhull(points_to_delete'));
				region_to_delete{index} = polyshape(points_to_delete(1,:),points_to_delete(2,:));
			end
		end
		% delete the area behind the intersection
		for i = 1:length(region_to_delete)
			poly_voronoi = subtract(poly_voronoi, region_to_delete{i});
		end
	
	% THERE ARE VISIBLE VERTICES OF THE OBSTACLE INSIDE THE VORONOI CELL
	else
		prev_point = [];
		next_point = [];
		projection_points = [];
		% check if a visible vertex is connected to a non visible vertex
		for i = 1:size(LO_vertices_copy,1)
			point_status = LO_vertices_copy(i,3);
			if point_status == 1
				if i == 1
					prev_point = LO_vertices_copy(end,:);
					next_point = LO_vertices_copy(i+1,:);
				elseif i == size(LO_vertices_copy,1)
					prev_point = LO_vertices_copy(i-1,:);
					next_point = LO_vertices_copy(1,:);
				else
					prev_point = LO_vertices_copy(i-1,:);
					next_point = LO_vertices_copy(i+1,:);
				end
				% find the projection point if either the previous or the next point is not visible
				if (prev_point(3) == 0 || next_point(3) == 0 || prev_point(3) == -1 || next_point(3) == -1)
						% elongate the visible point to find intersections
					elongated_point = LO_vertices_copy(i,1:2) + 1000*(LO_vertices_copy(i,1:2) - [0,0])/norm(LO_vertices_copy(i,1:2) - [0,0]);
					% find the projection point
					intersections_obstacle = intersect(LO.poly, [0,0;elongated_point]); % output given by row
					% remove the visible point itself from the intersection
					intersections_obstacle = intersections_obstacle(sum(abs(intersections_obstacle - LO_vertices_copy(i,1:2)).^2,2).^0.5 > 1e-4, :);
					% select the visible intersection
					distances = sum(abs(intersections_obstacle - [0,0]).^2,2).^0.5;
					[~,index] = min(distances);
					intersections_obstacle = intersections_obstacle(index,:);
					% check if the projection point is inside the voronoi cell
					intersection_voronoi = intersect(poly_voronoi, [0,0;elongated_point]); % output given by row
					% remove the robot position itself from the intersection
					intersection_voronoi = intersection_voronoi(sum(abs(intersection_voronoi - [0,0]).^2,2).^0.5 > 1e-4, :);

					if ~isempty(intersections_obstacle)
						if (norm(intersection_voronoi - [0;0]) < norm(intersections_obstacle - [0;0]))
							% add the point only if it is outside the obstacle
							if ~inpolygon(intersection_voronoi(1), intersection_voronoi(2), LO.poly.Vertices(:,1), LO.poly.Vertices(:,2))
								projection_points = [projection_points; intersection_voronoi];
							end
						else
							projection_points = [projection_points; intersections_obstacle];	
						end
					end
				end
			end
		end

		% find the visible intersection between the obstacle and the voronoi cell
		LO_vertices = LO.poly.Vertices;
		N_vertices = size(LO_vertices,1);
		visible_intersection = [];
		% add the first vertex at the end to close the polygon
		LO_vertices = LO_vertices([1:N_vertices,1],:);
		for i = 1:N_vertices
			intersection_voronoi = linexlines2D(poly_voronoi, LO_vertices(i,:), LO_vertices(i+1,:)); % output given by column
			% check if the intersection_voronoi is visible:
			% notice that the function linexlines2D returns the intersection between the voronoi cell and the obstacle but the 
			% output points are an approximation of the real ones.
			% So in order to verify if such points are visible or not we remove from the function "intersect" the points that may be 
			% equal the one given by the function linexlines2D
			for k = 1:size(intersection_voronoi,2)
				intersection_obstacle = intersect(LO.poly, [0,0;intersection_voronoi(1,k), intersection_voronoi(2,k)]); % output given by row
				distances = sum(abs(intersection_obstacle - [intersection_voronoi(1,k), intersection_voronoi(2,k)]).^2, 2).^0.5;
				intersection_obstacle(distances < 1e-4,:) = [];
				% If after removing "itself" there are still intesections with the obstacle then the intersection with voronoi is not visible
				% and then we proced with the reduction of the cell
				if isempty(intersection_obstacle)
					visible_intersection = [visible_intersection; intersection_voronoi(:,k)']; % output given by row
				end
			end
		end
		% join the matrices with: intersections btween voronoi and the obstacle, projections and visible vertices
		visible_points = [visible_intersection; projection_points; LO_vertices_copy(LO_vertices_copy(:,3) == 1, 1:2)];

		% order the points in anticlockwise order (if two points have the same angle then the closest one is the first)
		angles = atan2(visible_points(:,2), visible_points(:,1));
		[~,index] = sort(angles);
		angle = angles(index);
		visible_points = visible_points(index,:);
		for i = 1:size(visible_points,1) - 1
			if abs(angle(i+1) - angle(i)) < 1e-4
				if norm(visible_points(i+1,:) - [0,0]) < norm(visible_points(i,:)- [0,0])
					tmp = visible_points(i+1,:);
					visible_points(i+1,:) = visible_points(i,:);
					visible_points(i,:) = tmp;
				end
			end
		end
		plot(visible_points(:,1), visible_points(:,2), 'or');
		% TODO: move the intersections accordingly to the uncertainties
	end
end
% 		% find, if any, the intersection between the obstacle and the voronoi cell
% 		intersection = [];
%         pp = [];
% 		LO_vertices = LO.poly.Vertices;
% 		N_vertices = size(LO_vertices,1);
% 		% add the first vertex at the end to close the polygon
% 		LO_vertices = LO_vertices([1:N_vertices,1],:);
% 		for i = 1:N_vertices
% 			intersection = linexlines2D(poly_voronoi, LO_vertices(i,:), LO_vertices(i+1,:));
% 			pp = [pp, intersection];
%             plot(intersection(1,:),intersection(2,:),'og','MarkerFaceColor','g');
% 		end
        
% 		for i = 1:size(visible_points,1)
% 			plot(visible_points(i,1), visible_points(i,2), 'or');
%             pp = [pp, visible_points(i,:)'];
% 		end
% 	end
% end

% clf;
% hold on;
% grid on;
% axis equal;
% axis(10 * [-1 1 -1 1]);
% plot(poly_voronoi)
% % LO.plot();

