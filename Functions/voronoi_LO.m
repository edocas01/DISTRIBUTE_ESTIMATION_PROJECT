function [enlarged_LO] = voronoi_LO(LO, robot, max_semiaxis, param)
	enlarged_LO = [];
    poly_voronoi = robot.voronoi;
	x_r = robot.x(1);
	y_r = robot.x(2);
	x_e = robot.x_est(1);
	y_e = robot.x_est(2);
	

	%{

 
    ____                _                   _     _   _                                                             __   _   _            _     ___  
   / ___|___  _ __  ___| |_ _ __ _   _  ___| |_  | |_| |__   ___   _ __ ___   ___  __ _ ___ _   _ _ __ ___    ___  / _| | |_| |__   ___  | |   / _ \ 
  | |   / _ \| '_ \/ __| __| '__| | | |/ __| __| | __| '_ \ / _ \ | '_ ` _ \ / _ \/ _` / __| | | | '__/ _ \  / _ \| |_  | __| '_ \ / _ \ | |  | | | |
  | |__| (_) | | | \__ \ |_| |  | |_| | (__| |_  | |_| | | |  __/ | | | | | |  __/ (_| \__ \ |_| | | |  __/ | (_) |  _| | |_| | | |  __/ | |__| |_| |
   \____\___/|_| |_|___/\__|_|   \__,_|\___|\__|  \__|_| |_|\___| |_| |_| |_|\___|\__,_|___/\__,_|_|  \___|  \___/|_|    \__|_| |_|\___| |_____\___/ 
                                                                                                                                                     
 

	%}

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


	% Check if obstacle in cell

	if intersection.NumRegions > 0 % there is intersection
		% check if the are vertices of the obstacle inside the cell
		for i = 1 : size(LO.x,1)
			% if there are vertices of the obstacle inside the cell
			if inpolygon(LO.x(i,1), LO.x(i,2), poly_voronoi.Vertices(:,1), poly_voronoi.Vertices(:,2))
				% if I can see this point from the origin I use this point (the line connecting me and the point is inside the obstacle or not)
				if isempty(intersect(LO.poly, [x_r,y_r; LO.x(i,:)]))
					LO_vertices_copy(i,3) = 1;
					count_visible_points = count_visible_points + 1;
				else
					LO_vertices_copy(i,3) = 0;
				end
			end
		end


		% No obstalce in cell

		if count_visible_points == 0 
			intersection_voronoi = [];
			couples_to_delete = []; % couples of points to be delete
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
					intersection_obstacle = intersect(LO.poly, [x_r,y_r;intersection_voronoi(1,k), intersection_voronoi(2,k)]); % output given by row
					distances = sum(abs(intersection_obstacle - [intersection_voronoi(1,k), intersection_voronoi(2,k)]).^2,2).^0.5;
					intersection_obstacle(distances < 1e-5,:) = [];
					% If after removing "itself" there are still intesections with the obstacle then the intersection with voronoi is not visible
					% and then we proced with the reduction of the cell
					if isempty(intersection_obstacle)
						visible_intersection = [visible_intersection, intersection_voronoi(:,k)]; % output given by column
					end
				end
				
				% if the are visible intersections between the obstacle and the voronoi cell then delete the area behind
				if (~isempty(visible_intersection))
					if size(visible_intersection,2) ~= 2
						error("Problem with intersections between the obstacle and the voronoi cell");
					end
					couples_to_delete = [couples_to_delete; [visible_intersection(:,1)']]; % matrix n by 2
					couples_to_delete = [couples_to_delete; [visible_intersection(:,2)']]; % matrix n by 2
				end
			end
		

		% Vertices in voronoi

		else 
			
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
						angle = atan2(LO_vertices_copy(i,2) - y_r, LO_vertices_copy(i,1) - x_r);
						% if the angle is negative then add 2*pi
						if angle < 0
							angle = angle + 2*pi;
						end
						distance = norm(LO_vertices_copy(i,1:2) - [x_r,y_r]);
						th = 1e-10;
						% create a new ideal point with a small different angle
						ideal_point_min = [distance*cos(angle - th), distance*sin(angle - th)] + [x_r,y_r];
						ideal_point_plus = [distance*cos(angle + th), distance*sin(angle + th)] + [x_r,y_r];
						visible_point_min = intersect(LO.poly, [x_r,y_r;ideal_point_min]); % output given by row
						visible_point_plus = intersect(LO.poly, [x_r,y_r;ideal_point_plus]); % output given by row

						% If a point is visible and the other not then the correct one is the visible one
						% In the other cases we have to check the projections
						if isempty(visible_point_min) && ~isempty(visible_point_plus) % only the min is visible
							order = -100;
						elseif ~isempty(visible_point_min) && isempty(visible_point_plus) % only the plus is visible
							order = 100;
						else % both are visible
							% find the two projection points and take the farthest one 
							% min
							ideal_point_min = ideal_point_min + 1000*(ideal_point_min - [x_r,y_r])/norm(ideal_point_min - [x_r,y_r]);
							intersection_min = intersect(LO.poly, [x_r,y_r;ideal_point_min]); % output given by row
							distances = sum(abs(intersection_min - [x_r,y_r]).^2,2).^0.5;
							[~,index] = min(distances);
							if ~isempty(intersection_min)
								intersection_min = intersection_min(index,:);
							end
							% plus
							ideal_point_plus = ideal_point_plus + 1000*(ideal_point_plus - [x_r,y_r])/norm(ideal_point_plus - [x_r,y_r]);
							intersection_plus = intersect(LO.poly, [x_r,y_r;ideal_point_plus]); % output given by row
							distances = sum(abs(intersection_min - [x_r,y_r]).^2,2).^0.5;
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
						elongated_point = LO_vertices_copy(i,1:2) + 1000*(LO_vertices_copy(i,1:2) - [x_r,y_r])/norm(LO_vertices_copy(i,1:2) - [x_r,y_r]);
						% find the projection point
						intersections_obstacle = intersect(LO.poly, [x_r,y_r;elongated_point]); % output given by row
						% remove the visible point itself from the intersection
						intersections_obstacle = intersections_obstacle(sum(abs(intersections_obstacle - LO_vertices_copy(i,1:2)).^2,2).^0.5 > 1e-4, :);
						% select the visible intersection
						distances = sum(abs(intersections_obstacle - [x_r,y_r]).^2,2).^0.5;
						[~,index] = min(distances);
						intersections_obstacle = intersections_obstacle(index,:);
						% check if it is really visible
						% Check if a point between the projection point and the hiding point is inside the obstacle
						% If yes, then the projection point is not visible
						if ~isempty(intersections_obstacle)
							% create an ideal point a little bit closer to the robot from the projection point
							ideal_point = intersections_obstacle - 1e-4*(intersections_obstacle - [x_r,y_r])/norm(intersections_obstacle - [x_r,y_r]);
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
					intersection_obstacle = intersect(LO.poly, [x_r,y_r;intersection_voronoi(1,k), intersection_voronoi(2,k)]); % output given by row
					distances = sum(abs(intersection_obstacle - [intersection_voronoi(1,k), intersection_voronoi(2,k)]).^2, 2).^0.5;
					intersection_obstacle(distances < 1e-5,:) = [];
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
			angles = atan2(visible_points(:,2) - y_r, visible_points(:,1) - x_r);
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
			% remove the third column
			visible_points = visible_points(:,1:2);

			
			% test the points 2 by 2
			% if two points have more or less the same angle then do nothing
			% if two consecutive points "behind" them have not the obstacle then do nothing
			% copy the first point at the end to close the polygon
			visible_points = [visible_points; visible_points(1,:)];
			couples_to_delete = [];
			if size(visible_points,1) == 3
				couples_to_delete = [visible_points(1,1:2) ; visible_points(2,1:2)];
			else
				for i = 1:size(visible_points,1)-1
					% if two points have more or less the same angle then remove the area in every case
					angle_i = atan2(visible_points(i,2) - y_r, visible_points(i,1) - x_r);
					angle_i1 = atan2(visible_points(i+1,2) - y_r, visible_points(i+1,1) - x_r);
					if abs(angle_i - angle_i1) < 1e-4
						continue;
					end

					check_inside = [];
					% if two consecutive points "behind" them have not the obstacle then do nothing
					% create 5 points between the two points and check if there is the obstacle behind them
					% Obviously is not a perfect solution but it is a good approximation
					dir = (visible_points(i+1,:) - visible_points(i,:)) / norm(visible_points(i+1,:) - visible_points(i,:));
					dist = norm(visible_points(i+1,:) - visible_points(i,:));	
					check_inside = [check_inside; visible_points(i,:) 	+ 1e-3*dir];
					check_inside = [check_inside; visible_points(i,:) 	+ 0.25*dist*dir];
					check_inside = [check_inside; visible_points(i,:) 	+ 00.5*dist*dir];
					check_inside = [check_inside; visible_points(i,:) 	+ 0.75*dist*dir];
					check_inside = [check_inside; visible_points(i+1,:) - 1e-3*dir];
					% move the points a little bit further to the robot
					check_inside = check_inside + 1e-7*(check_inside - [x_r,y_r])./(sum(abs(check_inside - [x_r,y_r]).^2,2).^0.5);
					if inpolygon(check_inside(:,1),check_inside(:,2),LO.poly.Vertices(:,1),LO.poly.Vertices(:,2))
						couples_to_delete = [couples_to_delete; [visible_points(i,1:2) ; visible_points(i+1,1:2)]];
					end
					
				end
			end
		end


		%{

		
		__  __                                             _      _      _   _             
		|  \/  | ___  __ _ ___ _   _ _ __ ___     _      __| | ___| | ___| |_(_) ___  _ __  
		| |\/| |/ _ \/ _` / __| | | | '__/ _ \  _| |_   / _` |/ _ \ |/ _ \ __| |/ _ \| '_ \ 
		| |  | |  __/ (_| \__ \ |_| | | |  __/ |_   _| | (_| |  __/ |  __/ |_| | (_) | | | |
		|_|  |_|\___|\__,_|___/\__,_|_|  \___|   |_|    \__,_|\___|_|\___|\__|_|\___/|_| |_|
																							
		

		%}

		points_to_delete = [];
        couples_to_delete_before = couples_to_delete;
		index = 1;
		% measure the points and create the region to be deleted
		if size(couples_to_delete,1) > 1
		    for i = 1:2:size(couples_to_delete,1)
			    % perform the measure
				if i > 1
					if isequal(couples_to_delete(i,:), couples_to_delete_before(i-1,:));
						couples_to_delete(i,:) = couples_to_delete(i-1,:);
					else
						couples_to_delete(i,:) = (robot.H * (couples_to_delete(i,:)' - robot.x) + mvnrnd([0;0], robot.R_dist*0.5)')';
						couples_to_delete(i,:) = (couples_to_delete(i,:)' + robot.H * robot.x_est)';
					end
				else
					couples_to_delete(i,:) = (robot.H * (couples_to_delete(i,:)' - robot.x) + mvnrnd([0;0], robot.R_dist*0.5)')';
					couples_to_delete(i,:) = (couples_to_delete(i,:)' + robot.H * robot.x_est)';
				end
				if i+1 == size(couples_to_delete,1)
					if isequal(couples_to_delete(i+1,:),couples_to_delete_before(1,:))
						couples_to_delete(i+1,:) = couples_to_delete(1,:);
					else
						couples_to_delete(i+1,:) = (robot.H * (couples_to_delete(i+1,:)' - robot.x) + mvnrnd([0;0], robot.R_dist*0.5)')';
						couples_to_delete(i+1,:) = (couples_to_delete(i+1,:)' + robot.H * robot.x_est)';
					end
				else
					couples_to_delete(i+1,:) = (robot.H * (couples_to_delete(i+1,:)' - robot.x) + mvnrnd([0;0], robot.R_dist*0.5)')';
					couples_to_delete(i+1,:) = (couples_to_delete(i+1,:)' + robot.H * robot.x_est)';
				end
			    % define an area to delete for the first point
			    dir = couples_to_delete(i,:) - [x_e,y_e];
			    dir = dir/norm(dir);
			    % create a point 1000 meters behind the intersection
			    points_to_delete = [couples_to_delete(i,:) + 1000*dir; couples_to_delete(i,:)]; % matrix n by 2
				
			    % define an area to delete for the second point
			    dir = couples_to_delete(i+1,1:2) - [x_e,y_e];
			    dir = dir/norm(dir);
			    % create a point 1000 meters behind the intersection
			    points_to_delete = [points_to_delete; [couples_to_delete(i+1,1:2) + 1000*dir ; couples_to_delete(i+1,1:2)]];
    
			    % reorder the points to delete and create a polyshape
			    points_to_delete = points_to_delete(convhull(points_to_delete(:,1:2)),1:2);
			    region_to_delete{index} = polyshape(points_to_delete(:,1),points_to_delete(:,2));
			    index = index + 1;
		    end
		    total_region = region_to_delete{1};
		    % delete the area behind the intersection
		    for i = 2:length(region_to_delete)
			    total_region = union(region_to_delete{i},total_region);
		    end
			reg_obs = total_region;

		    % compute the reduction 
		    covariance = robot.R_dist*0.5 + robot.H * robot.P(1:2,1:2) * robot.H'; % covariance of the measurement
		    [V, D] = eig(covariance * param.coverage);
		    max_semiaxis_points = sqrt(max(diag(D))); % max semiaxis of the covariance for the measurement
		    total_reduction = max_semiaxis + robot.volume + max_semiaxis_points;
		    percentage = 1;
		    while true
			    tmp = polybuffer(total_region,total_reduction*percentage,'JointType','miter','MiterLimit',2);
			    % if the robot is iniside the augmented obstacle then reduce the percentage reduction
			    if inpolygon(x_e,y_e,tmp.Vertices(:,1),tmp.Vertices(:,2))
				    percentage = percentage - 0.1;
			    else
				    new_poly_voronoi = subtract(poly_voronoi, tmp);
				    break;
			    end
    
			    if percentage < 0.1
				    warning("Not so sure on the reduction of the voronoi cell");
				    new_poly_voronoi = subtract(poly_voronoi, total_region);
				    break;
			    end
			    
            end
			enlarged_LO = tmp;
        
		
	        Num_prev = 0;
            Num = 0;
            while new_poly_voronoi.NumRegions > 1
                Num_prev = new_poly_voronoi.NumRegions;
		        for i = 1:new_poly_voronoi.NumRegions
                    % Keep the region in which the robot is
		    	    tmp = rmboundary(new_poly_voronoi,i);
		    	    if inpolygon(x_e,y_e,tmp.Vertices(:,1),tmp.Vertices(:,2))
		    		    new_poly_voronoi = tmp;
		    		    break;
		    	    end
                end
                Num = new_poly_voronoi.NumRegions;
                if Num == Num_prev
                    break;
                end
            end
		    
    
%             if new_poly_voronoi.NumRegions > 1
%                 warning("Two regions in voronoi_LO")
% 				figure(3)
% 				hold on;
% 				grid on;
% 				axis equal;
% 				plot(x_e,y_e,'*r');
% 				plot(couples_to_delete(:,1),couples_to_delete(:,2),'*b');
% 				plot(poly_voronoi);
% 				LO.plot();
% 				plot(new_poly_voronoi);
%             end
    
		    %{
    
		    
     	     __  __       _           ____                          
		    |  \/  | __ _| | _____   / ___|___  _ ____   _______  __
		    | |\/| |/ _` | |/ / _ \ | |   / _ \| '_ \ \ / / _ \ \/ /
		    | |  | | (_| |   <  __/ | |__| (_) | | | \ V /  __/>  < 
		    |_|  |_|\__,_|_|\_\___|  \____\___/|_| |_|\_/ \___/_/\_\
																    
		    
    
		    %}
    
		    new_points = setdiff(new_poly_voronoi.Vertices,poly_voronoi.Vertices,'rows','stable');
		    % remove the points if they are Nan
		    % new_points(isnan(new_points(:,1)),:) = [];
		    index = 1;
		    index_visible = 0;
    
		    region_to_delete = [];
		    for i = 1:size(new_points,1)-1
			    % delete the area two by two points
			    % define an area to delete for the first point
			    dir = new_points(i,:) - [x_e,y_e];
			    dir = dir/norm(dir);
			    % create a point 1000 meters behind the intersection
			    points_to_delete = [new_points(i,:) + 1000*dir; new_points(i,:)]; % matrix n by 2
    
			    % define an area to delete for the second point
			    dir = new_points(i+1,1:2) - [x_e,y_e];
			    dir = dir/norm(dir);
			    % create a point 1000 meters behind the intersection
			    points_to_delete = [points_to_delete; [new_points(i+1,1:2) + 1000*dir ; new_points(i+1,1:2)]];
    
			    % reorder the points to delete and create a polyshape
			    points_to_delete = points_to_delete(convhull(points_to_delete(:,1:2)),1:2);
			    region_to_delete{i} = polyshape(points_to_delete(:,1),points_to_delete(:,2));
		    end
            if ~isempty(region_to_delete)
		        total_region = region_to_delete{1};
		        % delete the area behind the intersection
		        for i = 2:length(region_to_delete)
			        total_region = union(region_to_delete{i},total_region);
                end
		    final_voronoi = subtract(new_poly_voronoi, total_region);
		    % Update the voronoi cell
		    robot.voronoi = final_voronoi;
    	    end
    	end
    end

%     intersection = intersect(robot.voronoi, LO.poly);
%     if size(intersection.Vertices,1) > 0
%         figure(100)
%         hold on
%         axis equal
%         plot(x_e,y_e,'*r');
% 		plot(couples_to_delete(:,1),couples_to_delete(:,2),'*b');
%         plot(visible_points(:,1),visible_points(:,2),'og');
% 		plot(poly_voronoi);
% 		LO.plot();
% 		plot(robot.voronoi);
%     end
end