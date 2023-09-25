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
plot(0,0,'*k');

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



% check if the obstacle is inside the voronoi cell
count_visible_points = 0; % in order to se if there are visible vertices of the obstacle inside the voronoi cell
intersection = intersect(LO.poly, poly_voronoi); % find if there is an overlap between the obstacle and the voronoi cell
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
		% find the intersection between the obstacle and the voronoi cell
		% find visible intersection:
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
					% plot(intersection_voronoi(1,k),intersection_voronoi(2,k),'og','MarkerFaceColor','g');
				end
			end
			% if the are visible intersections between the obstacle and the voronoi cell then delete the area behind
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
	
		
			 

	else % THERE ARE VISIBLE VERTICES OF THE OBSTACLE INSIDE THE VORONOI CELL
		
		% projection_points is a matrix that contains the projection of the visible vertices of the obstacle on the obstacle itself
		% since the projection point comes from a visible vertex then they have the same angle. To avoid problem on that
		% the third column of the matrix is a flag to reconstruct the order of the points.
		%{
		projection_points:
		| x  | y | tag  |
		| -  | - | 100  | this point has to go after the associated visible vertex
		| -  | - | -100 | this point has to go before the associated visible vertex
		| -  | - | 0    | this means that the projection point is the point itself (there are not other points with the same angle)
		%}

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
					% String to verify the order of the angle
					order = -1;
					% check the angle in order to find the projection point (and give it a slightly different angle)
					angle = atan2(LO_vertices_copy(i,2), LO_vertices_copy(i,1));
					% if the angle is negative then add 2*pi
					if angle < 0
						angle = angle + 2*pi;
					end
					distance = norm(LO_vertices_copy(i,1:2));
					th = 1e-10;
					% create a new ideal point with a small different angle
                    ideal_point_min = [distance*cos(angle - th), distance*sin(angle - th)];
					ideal_point_plus = [distance*cos(angle + th), distance*sin(angle + th)];
					visible_point_min = intersect(LO.poly, [0,0;ideal_point_min]); % output given by row
					visible_point_plus = intersect(LO.poly, [0,0;ideal_point_plus]); % output given by row

					% If a point is visible and the other not then the correct one is the visible one
					% In the other cases we have to check the projections
					if isempty(visible_point_min) && ~isempty(visible_point_plus) % only the min is visible
						order = -100;
					elseif ~isempty(visible_point_min) && isempty(visible_point_plus) % only the plus is visible
						order = 100;
					else % both are visible
						% find the two projection points and take the farthest one 
						% min
						ideal_point_min = ideal_point_min + 1000*(ideal_point_min - [0,0])/norm(ideal_point_min - [0,0]);
						intersection_min = intersect(LO.poly, [0,0;ideal_point_min]); % output given by row
						distances = sum(abs(intersection_min - [0,0]).^2,2).^0.5;
						[~,index] = min(distances);
						if ~isempty(intersection_min)
							intersection_min = intersection_min(index,:);
						end
						% plus
						ideal_point_plus = ideal_point_plus + 1000*(ideal_point_plus - [0,0])/norm(ideal_point_plus - [0,0]);
						intersection_plus = intersect(LO.poly, [0,0;ideal_point_plus]); % output given by row
						distances = sum(abs(intersection_min - [0,0]).^2,2).^0.5;
						[~,index] = min(distances);
						if ~isempty(intersection_plus)
							intersection_plus = intersection_plus(index,:);
						end
						
						% take the farthest projection point
						if ~isempty(intersection_min) && ~isempty(intersection_plus)
							if norm(intersection_min) < norm(intersection_plus)
								order = 100;
							else
								order = -100;
							end
						else
							order = 0;
						end
					end

					% generate the projection point accordingly to the order
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
					% check if it is really visible
					% Check if a point between the projection point and the hiding point is inside the obstacle
					% If yes, then the projection point is not visible
					if ~isempty(intersections_obstacle)
						% create an ideal point a little bit closer to the robot from the projection point
						ideal_point = intersections_obstacle - 1e-4*(intersections_obstacle - [0,0])/norm(intersections_obstacle - [0,0]);
						% check if the ideal point is inside the obstacle
						inpolygon_ideal_point = inpolygon(ideal_point(1), ideal_point(2), LO.poly.Vertices(:,1), LO.poly.Vertices(:,2));
						if inpolygon_ideal_point
							% if the ideal point is inside the obstacle then the projection point is not visible
							continue;							
						else
							projection_points = [projection_points; [intersections_obstacle, order]];
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
					visible_intersection = [visible_intersection; [intersection_voronoi(:,k)',0]]; % output given by row
				end
			end
		end
		% join the matrices with: intersections between voronoi and the obstacle, projections and visible vertices
		visible_points = [visible_intersection; projection_points; [LO_vertices_copy(LO_vertices_copy(:,3) == 1, 1:2), zeros(size(LO_vertices_copy(LO_vertices_copy(:,3) == 1, 1:2),1),1)]];

		% order the points in anticlockwise order (if two points have the same angle then take the one accordingly to the flag)
		angles = atan2(visible_points(:,2), visible_points(:,1));
		% if an angle is negative then add 2*pi
		angles(angles < 0) = angles(angles < 0) + 2*pi;
		[~,index] = sort(angles);
		angles = angles(index);
		visible_points = visible_points(index,:);
		for i = 1:size(visible_points,1)-1
			if abs(angles(i) - angles(i+1)) < 1e-5
				flag_i = visible_points(i,3);
				flag_i1 = visible_points(i+1,3);
				if flag_i == 0 && flag_i1 == 0
					error("Problem in the order of the points");
				elseif ((flag_i == 0 && flag_i1 == 100) || (flag_i == -100 && flag_i1 == 0))
					continue;
				elseif ((flag_i == 0 && flag_i1 == -100) || (flag_i == 100 && flag_i1 == 0))
					tmp = visible_points(i+1,:);
					visible_points(i+1,:) = visible_points(i,:);
					visible_points(i,:) = tmp;
				end
			end			
		end

		for i = 1:size(visible_points,1)
			plot(visible_points(i,1), visible_points(i,2), 'or');
		end
		% remove the third column
		visible_points = visible_points(:,1:2);

		% TODO: move the intersections accordingly to the uncertainties

		% test the points 2 by 2
		% if two points have more or less the same angle then do nothing
		% if two consecutive points "behind" them have not the obstacle then do nothing
		% copy the first point at the end to close the polygon
		visible_points = [visible_points; visible_points(1,:)];
        points_to_delete = [];
		index = 1;
		for i = 1:size(visible_points,1)-1
			% if two points have more or less the same angle then remove the area in every case
			angle_i = atan2(visible_points(i,2), visible_points(i,1));
			angle_i1 = atan2(visible_points(i+1,2), visible_points(i+1,1));
			if abs(angle_i - angle_i1) < 1e-4
				continue;
			end

			% if two consecutive points "behind" them have not the obstacle then do nothing
			% create an ideal point in the "middle" of the two points but on the other side with respect to the robot 

			dir = (visible_points(i+1,:) - visible_points(i,:))/norm(visible_points(i+1,:) - visible_points(i,:));
			ideal_point = visible_points(i,:) + 1e-3*dir;
			% move the point a little bit further to the robot
			ideal_point = ideal_point + 1e-7*(ideal_point - [0,0])/norm(ideal_point - [0,0]);

			if inpolygon(ideal_point(1),ideal_point(2),LO.poly.Vertices(:,1),LO.poly.Vertices(:,2))
				% define an area to delete for the first point
				dir = visible_points(i,1:2) - [0,0];
				dir = dir/norm(dir);
				% create a point 1000 meters behind the intersection
				points_to_delete = [visible_points(i,1:2) + 1000*dir ; visible_points(i,1:2)]; % matrix 2 by n

				% define an area to delete for the second point
				dir = visible_points(i+1,1:2) - [0,0];
				dir = dir/norm(dir);
				% create a point 1000 meters behind the intersection
				points_to_delete = [points_to_delete; [visible_points(i+1,1:2) + 1000*dir ; visible_points(i+1,1:2)]];

				% reorder the points to delete and create a polyshape
				points_to_delete = points_to_delete(convhull(points_to_delete(:,1:2)),1:2);
				region_to_delete{index} = polyshape(points_to_delete(:,1),points_to_delete(:,2));
				plot(region_to_delete{index});
				index = index + 1;
            end

		end
	end
    % delete the area behind the intersection
	for i = 1:length(region_to_delete)
		poly_voronoi = subtract(poly_voronoi, region_to_delete{i});
    end
	
end


figure(2);
hold on;
grid on;
axis equal;
axis(10 * [-1 1 -1 1]);
plot(poly_voronoi)

LO.plot();

