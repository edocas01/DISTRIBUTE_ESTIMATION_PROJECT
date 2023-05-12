% This function is used to calculate the relative target consensus
function relative_target_consensous(robots, target, param)
	% Robots is a cell array of robots
	n = length(robots);
	% Number of consensous protocols messages
	m = param.MSG_PROTOCOL;
	F = cell(n,1);
	a = cell(n,1);
	% topology matrix
	A = zeros(n, n);
	
	for i = 1:n
		robots{i}.neighbors = [];
		count = 0;
		for j = 1:n
			% if robots j is in the communication radius of robot i
			% then then i can communicate with j
            if j == i 
                continue
            end
			robots_d =  norm(robots{i}.x - robots{j}.x);
			if robots_d <= robots{i}.ComRadius
				A(i, j) = 1;
				count = count + 1;
				robots{i}.neighbors = [robots{i}.neighbors, j];
			end
		end
		if count == 0
			warning("Robot " + i + " is not connected to the network");
		end

		% if the target can be measured by the robot
		dits_robot_target = norm(target.x - robots{i}.x);
		if dits_robot_target <= robots{i}.ComRadius
			% target in robot reference frame
			target_measure = robots{i}.H * (target.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
			% target world frame
			z = target_measure + robots{i}.H * robots{i}.x_est;
			% covariance matrix on the target estimate
			P_target_sensor = robots{i}.R_dist + robots{i}.H * robots{i}.P * robots{i}.H';
		else
			% the robot cannot measure the target so it uses the last estimate of the target
			% and the covariance matrix of the target estimate is set to a high value
			z = robots{i}.target_est;
			robots{i}.target_P = robots{i}.target_P * 10;
			if norm(robots{i}.target_P) >= norm(eye(2)*1000)
				P_target_sensor = eye(2)*1000;
			else
				P_target_sensor = robots{i}.target_P;
			end
		end
		
		% initialize the matrices for the maximum degree weighting
		F{i} = robots{i}.H' * inv(P_target_sensor) * robots{i}.H;
		a{i} = robots{i}.H' * inv(P_target_sensor) * z;
		
		robots{i}.target_est_hist_messages(:, 1) = inv(F{i}) * a{i};
		robots{i}.target_P_hist_messages{1} = inv(F{i});
	end
	
	D = A * ones(n,1);
	for k = 1:m
		 % Maximum Degree Weighting
		 FStore = F;
		 aStore = a;
		 for i=1:n
			for j=1:n
                % the i-th robot information is updated if j can send it to
                % him
				if A(j,i) == 1
					F{i} = F{i} + 1 / (1+max(D)) * (FStore{j} - FStore{i});
					a{i} = a{i} + 1 / (1+max(D)) * (aStore{j} - aStore{i});
				end
			end
			robots{i}.target_est_hist_messages(:, k+1) = inv(F{i}) * a{i};
			robots{i}.target_P_hist_messages{k+1} = inv(F{i});
		 end
	end
	% set in the robots the target position and the covariance matrix
	for i = 1:n
		robots{i}.target_est = inv(F{i})*a{i};
		robots{i}.target_P = inv(F{i});
	end

end