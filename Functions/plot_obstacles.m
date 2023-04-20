function plot_obstacles(obstacles)
	n = length(obstacles);
	for i = 1:n
		obstacles{i}.plot();
	end
end