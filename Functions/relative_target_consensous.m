% This function is used to calculate the relative target consensus
function relatvie_target_consensous(robots, target, param)
	% Robots is a cell array of robots
	n = length(robots);
	% Number of consensous protocols messages
	m = param.MSG_PROTOCOL;
	F = cell(n,1);
	a = cell(n,1);
	% topology matrix
	A = zeros(n, n);
	
	for i = 1:n
		for j = i+1:n
			% if the distance between the two robots is less than their communication range
			% then they can communicate with each other
			robots_d =  norm(robots{i}.x - robots{j}.x);
			A(i, j) = robots_d <= robots{i}.ComRadius;
		end
		% target in robot reference frame
		H = eye(2);
		target_measure = H * (target.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
		% target world frame
		z = target_measure + H * robots{i}.x_est;
		
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
		% tmp = robots{i}.target_P;
		% robots{i}.target_P = inv(robots{i}.target_P + F{i});
		% robots{i}.target_est = robots{i}.target_P * ((tmp) * robots{i}.target_est + a{i});
		robots{i}.target_est = inv(F{i})*a{i};
		robots{i}.target_P = inv(F{i});

	end

end