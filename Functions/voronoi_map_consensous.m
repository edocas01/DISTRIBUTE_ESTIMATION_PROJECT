function voronoi_map_consensous(param, robots, obstacles)
	N = length(robots);
	% move the neighbors according to:
	% - the uncertainty of j
	% - the uncertainty of i
	% - the encumbrance of i
	% NOTE: the neighbors have to be measured while the target is already estimated
	for i = 1:N
		modified_positions = [];
		% compute the max semiaxis of the uncertainty of i
		[~, eigenvalues] = eig(robots{i}.P*param.coverage);
		max_semiaxis = sqrt(max(diag(eigenvalues)));

		for j = 1:length(robots{i}.all_robots_pos)/2
			% continue if the robot is itself or if the robot has no information on the others inside all_robots_pos
			if i == j || norm(robots{i}.x_est - robots{i}.all_robots_pos(2*j-1:2*j)) > robots{i}.ComRadius*2
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
			% create the vector for the voronoi tesselation
			modified_positions = [modified_positions, z];
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
			% if the radius is smaller than the distance between the middle point and the agent
			if Rs^2 < norm(M - robots{i}.x_est)^2
				[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
				robots{i}.voronoi = polyshape(pointsx, pointsy);
			else
				dist_points = sqrt(Rs^2 - norm(M - robots{i}.x_est)^2); % distance between the middle point and the intersection points
				A = M + norm_dir*dist_points;      % circle-middle line intersection sx
				B = M - norm_dir*dist_points;      % circle-middle line intersection dx
				
				points = circle_sector(robots{i}.x_est(1), robots{i}.x_est(2), A, B); % points of the circular sector of interest
				robots{i}.voronoi = polyshape(points(:,1),points(:,2)); 
			end
		else
			% Save the positions of the agents and their neighbors in P (NOTE: the first row is the position of the agent itself)
			P(1,:) = robots{i}.x_est;
			for j = 1:len_neighbors
				P(j+1,:) = modified_positions(:,j);
			end
			% Compute the voronoi tesselation

			% NOTE:
			% - voronoi gives a set of points (also the "infinite" ones) but not the associations to the agents
			% - voronoin gives the associations to the agents but not the infinite points
			[vx,vy] = voronoi(P(:,1), P(:,2));
			[V,C] = voronoin(P);

			% remove infinite values in V (if there are any they are in the first row)
			if isinf(V(1,1)) || isinf(V(1,2)) 
				V(1,:) = []; % remove the first row of V
				C{1}(find(C{1} == 1)) = []; % remove the number 1 from C{1}
				C{1} = C{1} - 1;      % decrement all the other numbers of C{1} since we removed the first row of V
			end
		  
			% create a matrix of the points given by voronoi
			v = zeros(length(vx(1,:))*2,2);
			v = [vx(1,:)', vy(1,:)'; vx(2,:)', vy(2,:)']; 
			% remove the duplicate points
			v = unique(v, 'rows'); 
			
			% compare V and v to add the infinite points to V
            [~, ia] = setdiff(round(v,3), round(V,3), 'rows');
			%[~, ia] = setdiff(round(v,8), round(V,8), 'rows'); % a rounding is needed -> there are some small numerical issues
			inf_points = v(ia,:); % ia are the indices of the infinite points in v (points that are in v but not in V)
		
			% NOTE: the infinite points need to be elongated in order to perform the intersection with the sensing range
			% so we need to reconstruct the direction of the line checking the associated points in vx and vy
			% associate the infinite points to vx and vy and elongate them
			
			% Loop over the infinite points 
			for j = 1:length(inf_points(:,1))
				[r_inf,c_inf] = find(vx == inf_points(j,1)); % find the row and column of the infinite point in vx
				% if r_inf is not unique, check the y
				if length(r_inf) > 1
					[r_inf,c_inf] = find(vy == inf_points(j,2));
				end
				
				if r_inf == 1
					p_linked = [vx(2,c_inf), vy(2,c_inf)]; % point linked to the infinite point
				else
					p_linked = [vx(1,c_inf), vy(1,c_inf)];
				end
				% elongate the infinite point towards infinity
				inf_points(j, :) = inf_points(j,:) + (inf_points(j,:) - p_linked)/norm(inf_points(j,:) - p_linked)*Rs*100;
				V = [V; inf_points(j,:)]; % add the infinite point to V
			end
		
			% Associate the new points to the agents
			row_start = length(V(:,1)) - length(inf_points(:, 1)) + 1; % row where the infinite points start in V
			% Loop over the new points
			% for each point we check which is the closest agent
			% if the closest agent is the agent itself, we add the point to C{1}
			% NOTE: C is compute by each agent and the first row is always the agent itself so we
			% have to care only about the first row
			for j = row_start:length(V(:,1))
				V_dist = sum(abs(V(j,:)-P).^2,2).^0.5; % vector of distances between the considered point and the agents
				% V_dist is a vector of n elements where n is the number of agents
				V_index = find(V_dist == min(V_dist)); % check the closest agent
				% if the output is a vector of length 2, it means that the point is equidistant from 2 agents
				if length(V_index)== 2
					% if the point is close to the agent itself, add it to C{1}
					if length(find(V_index == 1)) == 1
						C{1} = [C{1}, j];
					end
				else % if the output is a vector of length 1, it means that the point is close to only one agent (numerical issues)
					V_dist(V_index) = max(V_dist); % set the first minimum to the maximum in order to not take it twice
					if length(find(V_index == 1)) == 1 % if the closest agent is the agent itself, add the point to C{1}
						C{1} = [C{1}, j];
						continue; % if the closest agent is the agent itself, go to the next point (no need to check the second minimum)
					end
					V_index = find(V_dist == min(V_dist)); % find the second minimum if the first one is not the agent itself
					if length(find(V_index == 1)) == 1
						C{1} = [C{1}, j];
					end
				end	
			end	  
			% create the polyshape of the cell
			k = convhull(V(C{1},:)); % take the points in a order such that they form a convex polygon
			% take the point of V associated to the agent itself in the order given by k
			poly_voronoi = polyshape(V(C{1}(k), 1), V(C{1}(k), 2));
			% create the polyshape of the sensing circle
			[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
			poly_circle = polyshape(pointsx,pointsy);
			% find the intersection between the two polyshapes
			robots{i}.voronoi = intersect(poly_circle, poly_voronoi);
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