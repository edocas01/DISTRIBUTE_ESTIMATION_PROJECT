% This does the centralized trilateration algorithm
function centralized_trilateration(robots, target, param)
	m = 100; % iteration of the non linear least square
	n = length(robots);
	A = zeros(n, n);
	for i = 1:n
		% find the neighbor of robot i
		for j = i+1:n
			% if the distance between the two robots is less than their communication range
			% then they can communicate with each other
			robots_d =  norm(robots{i}.x - robots{j}.x);
			A(i, j) = robots_d <= robots{i}.ComRadius;
		end
	end
	A = A + A';
	for i = 1:n 
		neighbors = find(A(i, :));
		neighbors = [i, neighbors]; % all indeces of robots in communication (inc)
		if length(neighbors) < 3
			continue;
		end
		
		% Non linear least square
		
		W = zeros(length(neighbors),length(neighbors));
		H = zeros(length(neighbors), 2); % linearization of h
		z = zeros(length(neighbors), 1); % measured distances
		h = zeros(length(neighbors), 1); % 
		R = W;
		for l = 1:length(neighbors)
			idx = neighbors(l);
			z(l) = norm(robots{idx}.x - target.x);
			h(l) = norm(robots{idx}.x_est - robots{idx}.target_est);
			H(l,1) = (robots{idx}.x_est(1) - robots{idx}.target_est(1)) / h(l);
			H(l,2) = (robots{idx}.x_est(2) - robots{idx}.target_est(2)) / h(l);
			R(l, l) = robots{idx}.R_dist;
		end
		W = R ;
		for k = 1:m
			robots{i}.target_est = robots{i}.target_est + inv(H' * W * H) * H' * W * (z - h);
			robots{i}.target_P = inv(H' * W * H);
			h(1) = norm(robots{i}.x_est - robots{i}.target_est);
			H(1,:) = (robots{i}.x_est - robots{i}.target_est) / h(1);
		end
	end

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
end