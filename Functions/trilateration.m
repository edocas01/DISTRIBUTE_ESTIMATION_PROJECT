% This function perform trilateration to find the position of the target for each robot
% each robot has to communicate with the other robots to find the position of the target and return the covariance
% matrix of the target position

function A = trilateration(robots, target, param)
	% Robots is a cell array of robots
	n = length(robots);
	% Number of consensous protocols messages
	m = param.MSG_PROTOCOL;
	F = cell(n,1);
	a = cell(n,1);
	% topology matrix
	A = zeros(n, n);
	for ll = 1:10
	for i = 1:n
		for j = i+1:n
			% if the distance between the two robots is less than their communication range
			% then they can communicate with each other
			robots_d =  norm(robots{i}.x - robots{j}.x);
			A(i, j) = robots_d <= robots{i}.ComRadius;
		end
		target_d = norm(robots{i}.x - target.x) + robots{i}.R_dist;
		target_d_est = norm(robots{i}.x_est - robots{i}.target_est);
		% jacobian matrix w.r.t. the target position
		H(1,1) = (robots{i}.x_est(1) - robots{i}.target_est(1)) / target_d_est;
		H(1,2) = (robots{i}.x_est(2) - robots{i}.target_est(2)) / target_d_est;
		% measurement of the distance between the robot and the target
		z = target_d;
		% initialize the matrices for the maximum degree weighting
		F{i} = H' * inv(robots{i}.R_dist + H * robots{i}.P * H') * H;
		a{i} = H' * inv(robots{i}.R_dist + H * robots{i}.P * H') * z;
	end	
	% make the matrix symmetric
	A = A + A';
	
	D = A * ones(n,1);
	for k = 1:m
		 % Maximum Degree Weighting
		 FStore = F;
		 aStore = a;
		 for i=1:n
			 for j=1:n
				
				 if A(i,j) == 1
					 F{i} = F{i} + 1 / (1+max(D)) * (FStore{j} - FStore{i});
					 a{i} = a{i} + 1 / (1+max(D)) * (aStore{j} - aStore{i});
				 end
			 end
		 end
	end
	% set in the robots the target position and the covariance matrix
	for i = 1:n
		tmp = robots{i}.target_P;
		robots{i}.target_P = inv(robots{i}.target_P + F{i});
		robots{i}.target_est = robots{i}.target_P * ((tmp) * robots{i}.target_est + a{i});
		% robots{i}.target_est = inv(F{i})*a{i};

	end

end
