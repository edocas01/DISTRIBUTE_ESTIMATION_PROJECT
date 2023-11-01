function [enlarged_LO, points_exit] = voronoi_map_consensous(param, robots, target, obstacles, LO)
	N = length(robots);
	% move the neighbors according to:
	% - the uncertainty of j
	% - the uncertainty of i
	% - the encumbrance of i
	% NOTE: the neighbors have to be measured while the target is already estimated
	for i = 1:N
		if robots{i}.robot_crash == true
			continue;
		end
		modified_positions = [];
		% compute the max semiaxis of the uncertainty of i
		[~, eigenvalues] = eig(robots{i}.P(1:2,1:2)*param.coverage);
		max_semiaxis = sqrt(max(diag(eigenvalues)));

		for j = 1:length(robots{i}.all_robots_pos)/2
			% continue if the robot is itself or if the robot cannot see
            % the other one I skip beacuse if I cannot see the other robot
            % surely it will not affect my voronoi cell
			if j == length(robots) + 1 
				tmp = target.x;
			else
				tmp = robots{j}.x;
			end

			if i == j || norm(robots{i}.x - tmp) > robots{i}.ComRadius
				continue;
			end
			% move the robot j in the closest point to the agent i according to the uncertainty of j
			z = moving_closer_point(robots{i}.x_est, robots{i}.all_robots_pos(2*j-1:2*j),...
			robots{i}.all_cov_pos(2*j-1:2*j,2*j-1:2*j), param.coverage);
			% move the robot j to consider the max uncertainty of i (max semiaxis of i)
			robots_d = norm(robots{i}.x_est - z);
			z = z + 2 * max_semiaxis * (robots{i}.x_est - z) / robots_d;	

			robots_d = norm(robots{i}.x_est - z);
			% if the vmax allows to exit from the "sicure cell" then reduce it of the volume
			if robots_d/2 < robots{i}.vmax * param.dt + robots{i}.volume
				z = z + 2 * robots{i}.volume * (robots{i}.x_est - z) / robots_d;
			end

			
			% if the point is behind the robot itself, move it in front of the robot of an epsilon
			if dot((robots{i}.all_robots_pos(2*j-1:2*j) - robots{i}.x_est),(z - robots{i}.x_est)) < 0
				if param.DEBUG
					warning("Robotj behind the roboti");
				end
				z = robots{i}.x_est + 1e-4 * (robots{i}.all_robots_pos(2*j-1:2*j) - robots{i}.x_est)/norm((robots{i}.all_robots_pos(2*j-1:2*j) - robots{i}.x_est));
			end

			% create the vector for the voronoi tesselation
			modified_positions = [modified_positions, z];
			
		end

		% add the obstacles for the voronoi tesselation
		for k = 1:length(obstacles)
			obstacle = obstacles{k}.x;
			obstacle_d = norm(robots{i}.x - obstacle);
				if obstacle_d <= robots{i}.ComRadius
					% obstacle in robot reference frame
					obstacle_measure = robots{i}.H * (obstacle - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
					% obstacle in world frame
					obstacle_measure = obstacle_measure + robots{i}.H * robots{i}.x_est;
					obstacle_covariance = robots{i}.R_dist + robots{i}.H * robots{i}.P(1:2,1:2) * robots{i}.H';
					
					% move the obstacle in the closest point to the agent i according to the uncertainty on the obstacle 
					z = moving_closer_point(robots{i}.x_est, obstacle_measure, obstacle_covariance, param.coverage);
					% move the obstacle according to the max uncertainty of i (max semiaxis of i)
					obstacle_d = norm(robots{i}.x_est - z);
					z = z + 2 * max_semiaxis * (robots{i}.x_est - z) / obstacle_d;
					
					obstacle_d = norm(robots{i}.x_est - z);
					% if the vmax allows to exit from the "sicure cell" then reduce it of the volume
					if obstacle_d/2 < robots{i}.vmax * param.dt + robots{i}.volume
						z = z + 2 * robots{i}.volume * (robots{i}.x_est - z) / obstacle_d;
					end

					% if the point is behind the robot itself, move it in front of the robot of an epsilon
					if dot((obstacle_measure - robots{i}.x_est),(z - robots{i}.x_est)) < 0
						if param.DEBUG
							warning("Obstacle behind the robot");
						end
						z = robots{i}.x_est + 1e-4 * (obstacle_measure - robots{i}.x_est)/norm((obstacle_measure - robots{i}.x_est));
					end

					% create the vector for the voronoi tesselation
					modified_positions = [modified_positions, z];
				end
		end
		
		% Initialization of the variables
		P = [];
		vx = [];
		vy = [];
		V = [];
		C = [];
		v = [];
		ia = [];
		inf_points = [];
		% Define the number of neighbors
		len_neighbors = size(modified_positions,2);
		% Define the admissible radius
		Rs = robots{i}.ComRadius/2;
		% reduce the radius of the agent to consider the uncertainty of the agent
		Rs = Rs - max_semiaxis;

		% if the robot can exit from the "sicure cell" then reduce the radius
		if robots{i}.vmax * param.dt + robots{i}.volume > Rs
			Rs = Rs - robots{i}.volume;
		end

		% Control the number of neighbors and manage the cases
		if len_neighbors == 0 % no other agents -> go with sensing range only
			% This case should never happen because the target is always a neighbor
			% (if a robot cannot see the target it uses the last estimate)
			[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
    		robots{i}.voronoi = polyshape(pointsx, pointsy);
		elseif len_neighbors == 1 % only one agent -> take the line in the middle of the agents
			dir = modified_positions(:,1) - robots{i}.x_est; % direction of the line from robot to neighbor
			dir = dir/norm(dir);                % normalization of the line
			norm_dir = [-dir(2); dir(1)];       % normal to dir (i.e. line in the middle of the agents)

			M =  mean([robots{i}.x_est, modified_positions(:,1)], 2); % middle point
			% find the intersection between the circle around the robot and a half-plane that contains the robot
			A = M + 100 * norm_dir;
			B = M - 100 * norm_dir;
			C = A - 100 * dir;
			D = B - 100 * dir;
			semi_plane = polyshape([A(1) B(1) D(1) C(1)], [A(2) B(2) D(2) C(2)]);
			[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
			poly_circle = polyshape(pointsx,pointsy);
			robots{i}.voronoi = intersect(poly_circle, semi_plane);
		else
			% Save the positions of the agents and their neighbors in P (NOTE: the first row is the position of the agent itself)
			P = [robots{i}.x_est modified_positions];
			if i == 1
				points_exit = P(:,2:end);
			end
			% Compute the voronoi tesselation
			x_min = -param.size_map*1.5;
			x_max = param.size_map*1.5;
			y_min = -param.size_map*1.5;
			y_max = param.size_map*1.5;

			[~, ~, vert] =  voronoiPolyhedrons(P, [x_min y_min], [x_max y_max]);
			k = convhull(vert{1}(1,:), vert{1}(2,:));
			poly_voronoi = polyshape(vert{1}(1,k), vert{1}(2,k)); % voronoi cell of the robot itself

			% create the polyshape of the sensing circle
			[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
			poly_circle = polyshape(pointsx,pointsy);
			
			% find the intersection between the two polyshapes
			robots{i}.voronoi = intersect(poly_circle, poly_voronoi);
		end
		
		% if ~inpolygon(robots{i}.x_est(1),robots{i}.x_est(2), robots{i}.voronoi.Vertices(:,1), robots{i}.voronoi.Vertices(:,2))
		% 	figure(140)
		% 	hold on
		% 	axis equal
		% 	plot(robots{i}.voronoi)
		% 	plot(robots{i}.x_est(1),robots{i}.x_est(2), 'r*')
		% 	plot(modified_positions(1,:), modified_positions(2,:), 'b*')
		% end
        
		% remove large obstacles
		for z = 1:length(LO)
			tmp = voronoi_LO(LO{z}, robots{i}, max_semiaxis, param);
			if i == 1
				enlarged_LO = tmp;
			end
		end

	end
end





%{

 
   ___       _                        _  	 _____                 _   _                 
  |_ _|_ __ | |_ ___ _ __ _ __   __ _| | 	|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
   | || '_ \| __/ _ \ '__| '_ \ / _` | | 	| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
   | || | | | ||  __/ |  | | | | (_| | | 	|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
  |___|_| |_|\__\___|_|  |_| |_|\__,_|_| 	|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
                                                                                      
 

%}

% compute the circular sector given the center and the two points
function [p_circle] = circle_sector(x_center, y_center, A, B)
	% This function reports the point of a circle
	
	step = 1e-2;
	% wrapTo2Pi is used to have the angle between 0 and 2*pi
	alpha_A = wrapTo2Pi(atan2(A(2) - y_center, A(1) - x_center)); 
	alpha_B = wrapTo2Pi(atan2(B(2) - y_center, B(1) - x_center));
	
	% A is the point at the left of the agent, and B at the right
	% take into account if the the alpha_B is smaller than alpha_A -> add 2*pi in order to have it bigger
	% NOTE: we want to go always from A to B in order to have the circular sector that include the agent
	if alpha_B < alpha_A 
	  alpha_B = alpha_B + 2*pi;
	end
	
	% compute the radius of the sensing range inside the function in order to avoid numerical issues
	R = norm([x_center y_center]' - A); % radius of the circle
	
	% build the arc
	th = alpha_A:step:alpha_B;
	p_circle(:, 1) = x_center + R*cos(th);
	p_circle(:, 2) = y_center + R*sin(th);
end

% move a pointj closer to another pointi with a given covariancej
function new_pj = moving_closer_point(p_i, p_j, cov_j, prob)
	
    [V, D] = eig(cov_j * prob);
	max_semiaxis = sqrt(max(diag(D)));
	
	dir = p_i - p_j;
	dir = dir/norm(dir);
	new_pj = p_j + dir * max_semiaxis;
end