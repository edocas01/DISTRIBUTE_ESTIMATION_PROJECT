% This function compute the inputs for the robots according to the controls
function [u, barycenter] = compute_control(robot,param)
	% compute the control for the robot if it is alive
	if robot.robot_crash == true
		u = [0;0];
		barycenter = robot.x_est;
	else
		% decide the control for the robot
		[objective, phi, RANDOM] = is_on_circle(robot, param);
		[barycenter, msh] = compute_centroid(robot, phi, objective, param);

		if RANDOM && norm(barycenter - robot.x_est) < 0.1
			figure(1000);clf; axis equal; grid on; 
			config;
			hold on
			robot.plot_real(all_markers, color_matrix, true);
            plot(robot.voronoi)
			plot(barycenter(1), barycenter(2), 'ok');
			plot(objective(1), objective(2), 'om');
            a=1;
		end

		if norm(barycenter - robot.x_est) == 0
			u = [0;0];
		else
			kp = 1/param.dt;
			% compute the control
			if  kp * norm(barycenter - robot.x_est) < robot.vmax
				u = kp * (barycenter - robot.x_est) * param.dt;
			else
				u = robot.vmax * param.dt * (barycenter - robot.x_est) / norm(barycenter - robot.x_est);
			end
		end
	end
    
end


%{

 
   ___       _                        _   _____                 _   _                 
  |_ _|_ __ | |_ ___ _ __ _ __   __ _| | |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
   | || '_ \| __/ _ \ '__| '_ \ / _` | | | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
   | || | | | ||  __/ |  | | | | (_| | | |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
  |___|_| |_|\__\___|_|  |_| |_|\__,_|_| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                                                                                      
 

%}


% If the robot is not on the circle it has to reach it, applying a certain policy:
% after the consensus:
% - if a robot start without seeing the target it moves versus one of his neighbors
% - if a robot start whithout seeing the target and without neighbors it moves randomly
% - if a robot start seeing the trget it moves towards the target
% - if a robot reaches the maximium covariance on the target it moves randomly
function [center, phi, RANDOM] = decide_target_barycenter(robot,param)
    radius = 0;
	RANDOM = false;
    % Set false the radius for the reaching of the target    
    robot.set_distance_radius = false;
	% if a robot is not seeing the target
	if (robot.all_robots_pos(end-1) > 1e4 && robot.all_robots_pos(end) > 1e4) || (robot.all_cov_pos(end-1,end-1) > 1000 || robot.all_cov_pos(end,end) > 1000)
		RANDOM = true; % DA RIMUOVERE
		if robot.count_random_step > 20 || robot.count_random_step == 0
            robot.count_random_step = 0;
			distances = sum(abs(robot.voronoi.Vertices - robot.x_est').^2,2).^0.5;
			[maxdist, idx] = max(distances);
			th = atan2(robot.voronoi.Vertices(idx,2)-robot.x_est(2),robot.voronoi.Vertices(idx,1)-robot.x_est(1));
			robot.random_direction = [cos(th); sin(th)];
		end
		robot.count_random_step = robot.count_random_step + 1;
		center = robot.x_est + robot.random_direction * robot.ComRadius;
		func = @(x,y,r,x_t,y_t) exp(-((x-x_t)^2 + (y-y_t)^2)); % KEEP the "4"
	else % The target estimate is updated
		center = robot.all_robots_pos(end-1:end);
		robot.set_distance_radius = true;
		radius = param.DISTANCE_TARGET;
		func = @(x,y,r,x_t,y_t) exp(-r/10*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2); % KEEP the "4"
	end
	% control on the circle around the target
	phi = @(x,y) func(x, y, radius, center(1), center(2));

end

% If the robot is on the circle it has to move in order to keep the equidistance from the other robots
function [center, phi] = decide_circle_barycenter(robot, param)
	robot.set_distance_radius = false;
	radius = param.DISTANCE_TARGET;
	tolerance = param.TOLERANCE_DISTANCE;
	target = robot.all_robots_pos(end-1:end);

	% convert the robot positions into a matrix
	neighbors_on_circle = reshape(robot.all_robots_pos,2,[])';
	neighbors_on_circle(robot.id,:) = [];
	neighbors_on_circle(end,:) = [];
	% check if the neighbors are on the circle
	distances = sum(abs(target' - neighbors_on_circle).^2,2).^0.5;
	neighbors_on_circle(distances < radius - tolerance | distances > radius + tolerance,:) = [];

	% if there are no sufficient neighbors on the circle, the robot stay still (at least 2)
	if size(neighbors_on_circle,1) < 1
		center = robot.all_robots_pos(end-1:end);
		robot.set_distance_radius = true;
		radius = param.DISTANCE_TARGET;
		func = @(x,y,r,x_t,y_t) exp(-r/10*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2); % KEEP the "4"
		phi = @(x,y) func(x, y, radius, center(1), center(2));
	else
		neighbors_on_circle(end+1,:) = robot.x_est';
		my_idx = length(neighbors_on_circle(:,1));
		% compute the angles of the robots wrt target in order to find the "order"
		angles = wrapTo2Pi(atan2(neighbors_on_circle(:,2) - target(2), neighbors_on_circle(:,1) - target(1)));
		[~,idx] = sort(angles);
		
		% find the closest robots on the circle:
		% (first component is the anticlockwise neighbor starting from the horizontal axis)
		my_order = find(idx == my_idx);
		if my_order == 1
			angle_neighbors = [angles(idx(end),:); angles(idx(2),:)];
		elseif my_order == length(idx)
			angle_neighbors = [angles(idx(end-1),:); angles(idx(1),:)];
		else
			angle_neighbors = [angles(idx(my_order-1),:); angles(idx(my_order+1),:)];
		end
		
		% compute the angular distance between the two neighbors
		% notice that the angle has to be taken in the correct order:
		% from the previos to the next robot
		if (angles(my_idx) > angle_neighbors(1))
			angle_distance(1) = angles(my_idx) - angle_neighbors(1);
		else
			angle_distance(1) = angles(my_idx) - angle_neighbors(1) + 2*pi;
		end

		if (angle_neighbors(2) > angles(my_idx))
			angle_distance(2) = angle_neighbors(2) - angles(my_idx);
		else
			angle_distance(2) = angle_neighbors(2) - angles(my_idx) + 2*pi;
		end

		% apply the control law imposing a new angle accordingly to the difference
		% between the two angle_distances

		if angle_distance(1) <= angle_distance(2)
			% the robot has to move anticlockwise
			new_angle = angles(my_idx) + (angle_distance(2) - angle_distance(1))/2;
		else
			% the robot has to move clockwise
			new_angle = angles(my_idx) - (angle_distance(1) - angle_distance(2))/2;
		end

		% compute the center (it has to the circumference)
		center = target + radius*[cos(new_angle); sin(new_angle)];
		Func = @(x,y,x_t,y_t) exp(-((x-x_t)^2 + (y-y_t)^2)); % KEEP the "4"
		phi = @(x,y) Func(x, y, center(1), center(2));
	end
end

% Decide if the robot is on the circle or it has still to reach it
function [center, phi, RANDOM] = is_on_circle(robot, param)
	radius = param.DISTANCE_TARGET;
	tolerance = param.TOLERANCE_DISTANCE;
	target = robot.all_robots_pos(end-1:end);
	RANDOM = false;
	if norm(robot.x_est - target) >= radius - tolerance && norm(robot.x_est - target) <= radius + tolerance
		[center, phi] = decide_circle_barycenter(robot,param);
	else
		[center, phi, RANDOM] = decide_target_barycenter(robot,param);
	end
end
