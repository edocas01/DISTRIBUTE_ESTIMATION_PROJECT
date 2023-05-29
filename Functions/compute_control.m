% This function compute the inputs for the robots according to the controls

% NOTE:
% after the consensus:
% - if a robot start without seeing the target it moves versus one of his neighbors
% - if a robot start whithout seeing the target and without neighbors it moves randomly
% - if a robot start seeing the target it moves towards the target
% - if a robot reaches the maximium covariance on the target it moves randomly

function [u, barycenter] = compute_control(robot,param)
	% if a robot is not seeing the target
	if (robot.all_robots_pos(end-1) > 1e4 && robot.all_robots_pos(end) > 1e4)
		% select a random neighbor
		neighbor = robot.all_robots_pos;
		neighbor(robot.id:robot.id+1) = [];
		neighbor(neighbor > 1e4) = [];
		if length(neighbor) > 0 % robot must have at least one good neighbour 
			k = randsample(1:2:length(neighbor)-1,1);
			center = neighbor(k:k+1);
		else % Moves randomly
			xmin = min(robot.voronoi.Vertices(:,1));
			xmax = max(robot.voronoi.Vertices(:,1));
			ymin = min(robot.voronoi.Vertices(:,2));
			ymax = max(robot.voronoi.Vertices(:,2));

			in = 0;
			while ~in
				x = xmin + (xmax-xmin)*rand();
				y = ymin + (ymax-ymin)*rand();
				in = inpolygon(x, y, robot.voronoi.Vertices(:,1), robot.voronoi.Vertices(:,2));
			end
			center = [x;y];
		end
	else % The target estimate is not updated and the uncertainty grows (target is lost)
		if robot.all_cov_pos(end-1,end-1) > 1000 || robot.all_cov_pos(end,end) > 1000
			xmin = min(robot.voronoi.Vertices(:,1));
			xmax = max(robot.voronoi.Vertices(:,1));
			ymin = min(robot.voronoi.Vertices(:,2));
			ymax = max(robot.voronoi.Vertices(:,2));

			in = 0;
			while ~in
				x = xmin + (xmax-xmin)*rand();
				y = ymin + (ymax-ymin)*rand();
				in = inpolygon(x, y, robot.voronoi.Vertices(:,1), robot.voronoi.Vertices(:,2));
			end
			center = [x;y];
		else % The target estimate is updated and the uncertainty is low
			center = robot.all_robots_pos(end-1:end);
		end
	end


		%% control on the circle around the target
		radius = param.DISTANCE_TARGET;
		func = @(x,y,r,x_t,y_t) exp(-r/10*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2); % KEEP the "4"
		phi = @(x,y) func(x, y, radius, center(1), center(2));
		[barycenter, msh] = compute_centroid(robot, phi, radius);

		% other controls ...

		kp = 1/param.dt;
		%% compute the control
		if  kp * norm(barycenter - robot.x_est) < robot.vmax
			u = kp * (barycenter - robot.x_est) * param.dt;
		else
			u = robot.vmax * param.dt * (barycenter - robot.x_est) / norm(barycenter - robot.x_est);
		end
end