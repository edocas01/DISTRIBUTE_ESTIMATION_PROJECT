% This function perform trilateration to find the position of the target for each robot
% each robot has to communicate with the other robots to find the position of the target and return the covariance
% matrix of the target position

function A = trilateration(robots,target)
	% Robots is a cell array of robots
	n = length(robots);
	% topology matrix
	A = zeros(n, n);
	for i = 1:n
		for j = i+1:n
			% if the distance between the two robots is less than the sum of their communication range
			% then they can communicate with each other
			d =  norm(robots{i}.x - robots{j}.x);
			A(i, j) = d <= robots{i}.ComRadius;
			
		end
	end
	% make the matrix symmetric
	A = A + A';

	% perform the trilateration with the topology matrix
	for i = 1:n
		% find the neighbors of the robot (the one tha can communicate with it)
		neighbors = find(A(i, :));
		% if the robot has no neighbors, then it cannot find the target
		if length(neighbors) < 2
			fprintf("Robot %d has no sufficient neighbors",i);
			continue;
		end
		% perform the trilateration
		R = ones(neighbors+1, neighbors+1)*0.5;
		% construct the measurement vector
		for i = 1:length(neighbors)+1
			z(i) = norm(robors{i}.x - target.x);
			H(i,:) = [1/z(i)*(target.x(1) - robots{i}.x(1)), 1/z(i)*(target.x(2) - robots{i}.x(2))];  
		end
		
		robot{i}.target_est = inv(H'*inv(COV)*H)*H'*inv(COV)*z';
		robot{i}.target_cov = inv(H'*inv(COV)*H);
	end
	

end


%% z = Hx + epsilon

% H is the measurement matrix
% x is the state vector
% epsilon is the measurement noise

d = sqrt((xtar - xrob)^2 + (ytar - yrob)^2);

% measurement matrix
H = [(xtar - xrob)/d, (ytar - yrob)/d];