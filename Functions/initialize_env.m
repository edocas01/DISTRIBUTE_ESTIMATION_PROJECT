% This function is used to generate the obstacles and the target with its trajectory
function [target, trajectory, u_trajectory, obstacles, robots] = initialize_env(param)
    config;
	trajectory = [];
    u_trajectory = [];
	obstacles = {};
	robots = {};
	x = [];
	y = [];

	fig_1 = figure(1); clf;
	sgtitle("Select points to create obstacles")
	hold on;
	grid on;
	axis equal;
	axis(param.size_map * [-1 1 -1 1]);
	idx = 1;

	% Set the obstacles
	while true
		[xi, yi ,button] = ginput(1);
		x = [x xi];
		y = [y yi];
		if ~isequal(button,1) % if enter is pressed
			break;
		end
		obstacles{idx} = OBSTACLE(xi,yi);
		plot(x, y, 'sk');
		idx = idx + 1;
	end
	print_title("Acquired obstacles",param.title_flags);
	
	sgtitle("Select points to create trajectory")
	% Set the target trajectory
	x = [];
	y = [];
	while true
		[xi, yi ,button] = ginput(1);
		x = [x xi];
		y = [y yi];
		if ~isequal(button,1) % if enter is pressed
			break;
		end
		plot(x, y, '--ro');
	end
	
	print_title("Acquired trajectory", param.title_flags);
	trajectory = [x;y];
	n = length(x);
	
	% inputs for the target
	for i = 1:n-1
		% define the initial and final points
		x_in = [x(i); y(i)];
		x_fin = [x(i+1); y(i+1)];
		% compute distance from the point to the next point
		dist = norm(x_fin - x_in);
		% compute the number of steps as the distance divided by the time step
		N_steps = round(dist / (param.dt*param.vmax_target));
		% compute the velocity
		for j = 1:N_steps
			% the iput is equal to the velocity for a certain number of steps
			u_trajectory = [u_trajectory, (x_fin - x_in)/N_steps];
        end
    end
	
	% generate the target
	target = TARGET([x(1),y(1)]);
	
	idx = 1;
	% Set the robots
	sgtitle("Select points to create robots")
	
	while true
		[xi, yi ,button] = ginput(1);
		if ~isequal(button,1) % if enter is pressed
			break;
		end
		robots{idx} = ROBOT([xi;yi], idx, 'linear', param);
		robots{idx}.plot_real(all_markers, color_matrix, true);
		idx = idx + 1;
		if idx == param.N +1
			break;
		end
	end
	print_title("Acquired robots positions", param.title_flags);
	
	close(fig_1);
end
