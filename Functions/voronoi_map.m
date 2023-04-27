% This function computes the voronoi map of the robots
% It is possible to perform the global voronoi map and the local voronoi map
function [vx, vy] = voronoi_map(robots, obstacles)
	% number of robots
	n = length(robots);
	adiacent_robots = zeros(n,n);

	% number of obstacles
	m = length(obstacles);
	adiacent_obstacles = zeros(n,m);

	% compute the adiacent robots and obstacles for each robot
	for i = 1:n
		for j = 1:n
			% if robots j is in the communication radius of robot i
			% then then i can communicate with j
            if j == i 
                continue
            end
			robots_d =  norm(robots{i}.x - robots{j}.x);
			if robots_d <= robots{i}.ComRadius
				adiacent_robots(i,j) = 1;
			end
		end

		for k = 1:m
			% if obstacle k is in the communication radius of robot i
			% then then i can communicate with k
			obstacles_d =  norm(robots{i}.x - obstacles{k}.x);
			if obstacles_d <= robots{i}.ComRadius
				adiacent_obstacles(i,k) = 1;
			end
		end
	end
	
	
end