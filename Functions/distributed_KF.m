% Distributed Kalman Filter
function distributed_KF(robots, target, param)
	% Robots is a cell array of robots
	n = length(robots);
	% Number of consensous protocols messages
	m = param.MSG_PROTOCOL;
	F = cell(n,1);
	a = cell(n,1);
	% topology matrix
	A = zeros(n, n);


	for i = 1:n
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
			end
		end
		if count == 0
			warning("Robot " + i + " is not connected to the network");
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
	
	D = A * ones(n,1);

	% Compute F and a
	for k = 1:m
		 FStore = F;
		 aStore = a;
		 for i=1:n
			 for j=1:n
				
				if A(j,i) == 1
					F{i} = F{i} + 1 / (1+max(D)) * (FStore{j} - FStore{i});
					a{i} = a{i} + 1 / (1+max(D)) * (aStore{j} - aStore{i});
				end
			end
			robots{i}.target_est_hist_messages(:, k+1) = inv(F{i}) * a{i};
			robots{i}.target_P_hist_messages{k+1} = inv(F{i});
		 end
	end

	% Distributed Kalman Filter
	for i = 1:n
		% Prediction
		targ_P_prior = robots{i}.target_P;
		targ_est_prior = robots{i}.target_est;
		% Update
		robots{i}.target_P = inv(targ_P_prior + n * F{i}) ;
		robots{i}.target_est = robots{i}.target_P * (targ_P_prior * targ_est_prior + n * a{i});
	end


end