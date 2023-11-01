function metrics = compute_metrics(results, param)
	% function to compute metrics for the simulation
	% find robot on the circle
	N = length(results{1}.R);
	metrics = cell(1,N);
	radius = param.DISTANCE_TARGET;
	tolerance = param.TOLERANCE_DISTANCE;
	for i = 1:length(results)
		% matrix for robot position 
		for j = 1:N
			robot_pos(:,j) = results{i}.R{j}.x; % 2 x N
			robot_id(j) = results{i}.R{j}.id;
		end
		T_pos = results{i}.T.x; % 2 x 1

		% compute error on distance
		for j = 1:N
			% compute error on distance
			metrics{j}.err_dist(i) = norm(robot_pos(:,j) - T_pos) - radius;
		end
		
		% compute robot on the circle
		robot_id = robot_id(sum((robot_pos - T_pos).^2,1).^0.5 >= radius - tolerance & sum((robot_pos - T_pos).^2,1).^0.5 <= radius + tolerance);
		robot_pos = robot_pos(:,(sum((robot_pos - T_pos).^2,1).^0.5 >= radius - tolerance & sum((robot_pos - T_pos).^2,1).^0.5 <= radius + tolerance));

		% compute the angles of the robots
		if size(robot_pos,2) > 1
			angles = wrapTo2Pi(atan2(robot_pos(2,:) - T_pos(2), robot_pos(1,:) - T_pos(1)));
			[angles, idx] = sort(angles, 'ascend');
			robot_id = robot_id(idx);
			robot_pos = robot_pos(:,idx);

			for j = 1:length(robot_id)
				if j == 1
					delta_prev = min(abs(angles(j) - angles(end)), 2*pi - abs(angles(j) - angles(end))); 	
				else
					delta_prev = min(abs(angles(j) - angles(j-1)), 2*pi - abs(angles(j) - angles(j-1))); 	
				end

				if j == length(robot_id)
					delta_next = min(abs(angles(j) - angles(1)), 2*pi - abs(angles(j) - angles(1))); 	
				else
					delta_next = min(abs(angles(j) - angles(j+1)), 2*pi - abs(angles(j) - angles(j+1))); 	
				end
				
				metrics{robot_id(j)}.err_angles(i) = delta_next - delta_prev;
			end
			
			remaning_index = setdiff(1:N, robot_id, 'stable');
			for j = 1:length(remaning_index)
				metrics{remaning_index(j)}.err_angles(i) = 100;
			end
			
		else
			for j = 1:N
				metrics{j}.err_angles(i) = 100;
			end
		end
	end
end