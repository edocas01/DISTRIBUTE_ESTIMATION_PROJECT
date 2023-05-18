% This function is used to calculate the relative target consensus
function [z, cov] = relative_general_consensous(robots, target, param)
	% Robots is a cell array of robots
	n = length(robots);
	% Number of consensous protocols messages
	m = param.MSG_PROTOCOL;
	F = cell(n,1);
	a = cell(n,1);
	% topology matrix
	A = zeros(n, n);
	
	z = cell(n,1); % It will be 2 * (n+1) x 1 for each robot
	cov = cell(n,1); % It will be 2 * (n+1) x 2 * (n+1) for each robot
	
	% for i = 1:n
	% 	z{i} = [];
	% 	cov{i} = [];
	% end

	% Se non c'è un numero minimo di agenti che riescono a vedersi, l'algoritmo non funziona
	for i = 1:n
		robots{i}.neighbors = [];
		robots{i}.all_robots_pos = [zeros(2 * n, 1)];
		robots{i}.all_cov_pos = [zeros(2 * n, 2 * n)];
		count = 0;

		for j = 1:n
			% if robots j is in the communication radius of robot i
			% then then i can communicate with j
            if j ~= i % Robot i provides the position and covariance matrix of robot j
				robots_d =  norm(robots{i}.x - robots{j}.x);
				if robots_d <= robots{i}.ComRadius 
					% Robot i sees robot j and measures it

					A(i, j) = 1;
					count = count + 1;
					% robots{i}.neighbors = [robots{i}.neighbors, j];

					% robotj in roboti reference frame
					robot_measure = robots{i}.H * (robots{j}.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
					% robotj in world frame
					robot_j_world = robot_measure + robots{i}.H * robots{i}.x_est;

					robots{i}.all_robots_pos(2*j-1:2*j, 1) = robot_j_world;
					robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.R_dist + robots{i}.H * robots{i}.P * robots{i}.H';
				else 
					% Robot i cannot see robot j and uses the last estimate of robot j
					% and the covariance matrix of the robot estimate is set to a high value

					robots{i}.all_robots_pos(2*j-1:2*j, 1) = robots{i}.all_robots_pos(2*j-1:2*j, 1);
					
					if norm(robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) * 10) >= norm(eye(2)*1000)
						robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = eye(2)*1000;
					else
						robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) * 10;
					end
                end
			else % Robot i provides its own position and covariance matrix
				robots{i}.all_robots_pos(2*j-1:2*j, 1) = robots{i}.GPS_measurement();
				robots{i}.all_cov_pos(2*j-1:2*j, 2*j-1:2*j) = robots{i}.R_gps;
			end
		end

		if count == 0
			warning("Robot " + i + " cannot send messages to any other robot");
		end

		% % if the target can be measured by the robot
		% dist_robot_target = norm(target.x - robots{i}.x);
		% if dist_robot_target <= robots{i}.ComRadius
		% 	% target in robot reference frame
		% 	target_measure = robots{i}.H * (target.x - robots{i}.x) + mvnrnd([0;0], robots{i}.R_dist)';
		% 	% target world frame
		% 	zz = target_measure + robots{i}.H * robots{i}.x_est;
		% 	% covariance matrix on the target estimate
		% 	P_target_sensor = robots{i}.R_dist + robots{i}.H * robots{i}.P * robots{i}.H';
		% else
		% 	% the robot cannot measure the target so it uses the last estimate of the target
		% 	% and the covariance matrix of the target estimate is set to a high value
		% 	zz = robots{i}.target_est;
		% 	robots{i}.target_P = robots{i}.target_P * 10;
		% 	if norm(robots{i}.target_P) >= norm(eye(2)*1000)
		% 		P_target_sensor = eye(2)*1000;
		% 	else
		% 		P_target_sensor = robots{i}.target_P;
		% 	end
		% end

		% z{i} = [z{i}; zz];
		% % z(2*(n+1)-1:2*(n+1),1,i) = zz;
		% cov{i} = blkdiag(cov{i}, P_target_sensor);
		% cov(2*(n+1)-1:2*(n+1),2*(n+1)-1:2*(n+1),i) = P_target_sensor;
		H = eye(size(robots{i}.all_cov_pos));

		% initialize the matrices for the maximum degree weighting
		F{i} = H' * inv(robots{i}.all_cov_pos) * H;
		a{i} = H' * inv(robots{i}.all_cov_pos) * robots{i}.all_robots_pos;

		% robots{i}.target_est_hist_messages(:, 1) = inv(F{i}) * a{i};
		% robots{i}.target_P_hist_messages{1} = inv(F{i});
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
		robots{i}.all_robots_pos = inv(F{i}) * a{i};
		robots{i}.all_cov_pos = inv(F{i});
	end

end