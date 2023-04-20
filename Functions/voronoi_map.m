function [vx, vy] = voronoi_map(robots, obstacles)
	pos = zeros(2, length(robots));
	
	for i = 1:length(robots)
		pos(:,i) = robots{i}.x_est;
	end

	% Consider the communication range

	% Create the Voronoi map
	[vx, vy] = voronoi(pos(1, :), pos(2, :));
    
	

	
	
end
