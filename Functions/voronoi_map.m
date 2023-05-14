function voronoi_map(param, robots, obstacles)
	N = length(robots);
	% If the neighbors are not initialized, initialize them:
	if strcmp(robots{1}.neighbors, 'init')
		for i = 1:N
			robots{i}.neighbors = [];
			for j = 1:N
				% if robots j is in the communication radius of robot i
				% then then i can communicate with j
				if j ~= i
					robots_d =  norm(robots{i}.x - robots{j}.x);
					if robots_d <= robots{i}.ComRadius
						robots{i}.neighbors = [robots{i}.neighbors, j];
					end
				end
			end
		end
	end
	
	% insert in the robot.neighbors_pos (2, neighbours + 1) the position of the neighbors and the target at the end
	% move the neighbors according to:
	% - the uncertainty of j
	% - the uncertainty of i
	% - the encumbrance of i
	% NOTE: the neighbors have to be measured while the target is already estimated
	Delta = zeros(N,1);
	for i = 1:N 
		robots{i}.neighbors_pos = []; 
		for j = 1:length(robots{i}.neighbors) + 1 % to insert also the target
			if j <= length(robots{i}.neighbors)
				% Perform the measure on the neighbor
				% neighbor in agent reference frame
				neighbor_measure = (robots{robots{i}.neighbors(j)}.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
				% neighbor in world frame
				z = neighbor_measure + robots{i}.x_est;
				cov = robots{i}.R_dist + robots{i}.P;
				% if the neighbor can communicate with the agent then they can mean their estimate 
				robots_d = norm(robots{i}.x - robots{robots{i}.neighbors(j)}.x);
				if robots_d <= robots{robots{i}.neighbors(j)}.ComRadius
					% bayesian mean between the agent and the neighbor estimate on neighbor position
					z2 = robots{robots{i}.neighbors(j)}.x_est;
					cov2 = robots{robots{i}.neighbors(j)}.P; 
					z = [z;z2];
					H = [eye(2); eye(2)];
					cov = [cov, zeros(2); zeros(2), cov2];
					% fused information
					z = inv(H'*inv(cov)*H)*H'*inv(cov)*z;
					% fused covariance
					cov = inv(H'*inv(cov)*H);
				end
			else
				z = robots{i}.target_est;
				cov = robots{i}.target_P;
			end
			% move the robot j in the closest point to the agent i according to the uncertainty of j
			[~,z] = moving_closer_point(robots{i}.x_est, z, cov, 3);
			% move the robot j to consider the max uncertainty of i (max semiaxis of i)
			[~, eigenvalues] = eig(robots{i}.P*3);
			max_semiaxis = sqrt(max(diag(eigenvalues)));
			Delta(i) = robots{i}.volume + max_semiaxis;

			robots{i}.neighbors_pos(:,j) = z;		
			% if the vmax allows to exit from the "sicure cell" then reduce it
			robots_d = norm(robots{i}.x_est - z);
			if robots_d/2 < robots{i}.vmax * param.dt + Delta(i)
				robots{i}.neighbors_pos(:,j) = z + 2 * Delta(i) * (robots{i}.x_est - z) / robots_d;
			end
		end
	end

	for i = 1:N
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
		len_neighbors = length(robots{i}.neighbors_pos(1,:));
		% Define the admissible radius
		Rs = robots{i}.ComRadius/2;
		% if the robot can exit from the "sicure cell" then reduce the radius
		if robots{i}.vmax * param.dt + Delta(i) > Rs
			Rs = Rs - Delta(i);
		end

		% Control the number of neighbors and manage the cases
		if len_neighbors == 0 % no other agents -> go with sensing range only
			% This case should never happen because the target is always a neighbor
			% (if a robot cannot see the target it uses the last estimate)
			[pointsx, pointsy] = Circle(robots{i}.x_est(1), robots{i}.x_est(2), Rs);
    		robots{i}.voronoi = polyshape(pointsx, pointsy);
		elseif len_neighbors == 1 % only one agent -> take the line in the middle of the agents
			dir = robots{i}.neighbors_pos(:,1) - robots{i}.x_est; % direction of the line from robot to neighbor
			dir = dir/norm(dir);                % normalization of the line
			norm_dir = [-dir(2); dir(1)];       % normal to dir (i.e. line in the middle of the agents)

			M =  mean([robots{i}.x_est, robots{i}.neighbors_pos(:,1)], 2); % middle point
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
				P(j+1,:) = robots{i}.neighbors_pos(:,j);
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
			[~, ia] = setdiff(round(v, 6), round(V,6), 'rows'); % a rounding is needed -> there are some small numerical issues
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
function [point_ellipse, new_pj] = moving_closer_point(p_i, p_j, cov_j, prob)
	
	% find the ellipse around pj with covariance covj and probability prob
    [V, D] = eig(cov_j * prob);
    t = linspace(0, 2*pi, 30);
    point_ellipse = [p_j(1); p_j(2)] + (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];

	% find the closest point on the ellipse to pi
	p_i = [p_i(1); p_i(2)];
	[~,idx] = min(sum((point_ellipse - p_i).^2,1));
	new_pj = point_ellipse(:,idx);
end