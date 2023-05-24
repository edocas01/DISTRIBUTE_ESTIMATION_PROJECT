% This function is used to calculate the relative target consensus
function relative_general_consensous(robots, target, param)
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
		robots{i}.all_robots_pos = [zeros(2 * n, 1)];
		robots{i}.all_cov_pos = [zeros(2 * n, 2 * n)];
		count = 0;
		for j = 1:n
            if j ~= i
			% if robots j is in the communication radius of robot i
			% then then i can communicate/measure with j
				robots_d =  norm(robots{i}.x - robots{j}.x);
				if robots_d <= robots{i}.ComRadius 
					% Robot i sees robot j and measures it

					A(i, j) = 1;
					count = count + 1;
					% robots{i}.neighbors = [robots{i}.neighbors, j];

					% robotj in roboti reference frame
					robot_measure = robots{i}.H * (robots{j}.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
					% robotj in world frame
					robots{i}.all_robots_pos(2*j-1:2*j, 1) = robot_measure + robots{i}.H * robots{i}.x_est;
					robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.R_dist + robots{i}.H * robots{i}.P * robots{i}.H';
					
				else
					% the roboti cannot measure the robotj so it uses the last estimate of the robot
					% and the covariance matrix of the robot estimate is set to a high value
					robots{i}.all_robots_pos(2*j-1:2*j) = robots{i}.all_robots_pos(2*j-1:2*j);
					if norm(robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) * 5) >= norm(eye(2)*1000)
						robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = eye(2)*1000;
					else
						robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) * 5;
					end
				end
			else
				% the robot insert its own position in the matrix
				robots{i}.all_robots_pos(2*j-1:2*j, 1) = robots{i}.x_est;
				robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.P;
			end
		end

		if count == 0
			warning("Robot " + i + " cannot send messages to any other robot");
		end

		% if the target can be measured by the robot
		dits_robot_target = norm(target.x - robots{i}.x);
		if dits_robot_target <= robots{i}.ComRadius
			% target in robot reference frame
			target_measure = robots{i}.H * (target.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
			% target world frame
			robots{i}.all_robots_pos(end-1:end, 1) = target_measure + robots{i}.H * robots{i}.x_est;
			% covariance matrix on the target estimate
			robots{i}.all_cov_pos(end-1:end,end-1:end) = robots{i}.R_dist + robots{i}.H * robots{i}.P * robots{i}.H';
		else
			% the robot cannot measure the target so it uses the last estimate of the target
			% and the covariance matrix of the target estimate is set to a high value
			robots{i}.all_robots_pos(end-1:end, 1) = robots{i}.all_robots_pos(end-1:end, 1);
			if norm(robots{i}.all_cov_pos(end-1:end,end-1:end) * 10) >= norm(eye(2)*1000)
				robots{i}.all_cov_pos(end-1:end,end-1:end) = eye(2)*1000;
			else
				robots{i}.all_cov_pos(end-1:end,end-1:end) = robots{i}.all_cov_pos(end-1:end,end-1:end) * 10;
			end
		end

		H = eye(2*(n+1));

		% initialize the matrices for the maximum degree weighting
		F{i} = H' * inv(robots{i}.all_cov_pos) * H;
		a{i} = H' * inv(robots{i}.all_cov_pos) * robots{i}.all_robots_pos;

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
		 end
	end
	% set in the robots the target position and the covariance matrix
	for i = 1:n
		robots{i}.all_robots_pos = inv(F{i}) * a{i};
		robots{i}.all_cov_pos = inv(F{i});
	end

end