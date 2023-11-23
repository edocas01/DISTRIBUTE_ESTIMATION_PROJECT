% This function is used to generate the obstacles and the target with its trajectory
function [target, trajectory, u_trajectory, obstacles, large_obstacles, robots] = initialize_env(param)
    config;
	trajectory = [];
    u_trajectory = [];
	obstacles = {};
	large_obstacles = {};
	robots = {};
	x = [];
	y = [];

	fig_1 = figure(1); clf;
	% Set the large obstacles
	sgtitle("Select points to create obstacles")
	hold on;
	grid on;
	axis equal;
	axis(param.size_map * [-1 1 -1 1]);
	
    size_map = param.size_map - 0.08;
    th = 0.01;
    width = 1;

    X = [-size_map size_map];
    X = [X; [-size_map -size_map]];
    X = [X; [size_map -size_map]];
    X = [X; [size_map size_map+th]];
    X = [X; [-size_map-30 size_map+th]];
    X = [X; [-size_map-30 size_map+th+width]];
    X = [X; [size_map+width size_map+th+width]];
    X = [X; [size_map+width -size_map-width]];
    X = [X; [-size_map-width -size_map-width]];
    X = [X; [-size_map-width size_map]];

	large_obstacles{1} = LARGE_OBSTACLE(X);
    large_obstacles{1}.plot();
	idx = 2;
	FIRST = false;
	first_time = true;
	exit = false;
	while true
		x = [];
		y = [];
		X = [];
		while true
			if ~FIRST
				[xi, yi ,button] = ginput(1);
            end

			if first_time
				if ~isequal(button,1) % if enter is pressed
					exit = true;
					break;
				end
			end
			first_time = false;

			if size(X,1) > 2
           		if ~isequal(button,1) % if enter is pressed
					break;
				end
			end
			FIRST = false;
			x = [x xi];
			y = [y yi];
			X = [x' y'];
			plot(x, y, '--ko');
		end 
		
		if exit
			break;
		end
		
		large_obstacles{idx} = LARGE_OBSTACLE(X);
		clf;
		sgtitle("Select points to create obstacles")
		hold on;
		grid on;
		axis equal;
		axis(param.size_map * [-1 1 -1 1]);
		for i = 1:length(large_obstacles)
			large_obstacles{i}.plot();
		end
		idx = idx + 1;
		
		[xi,yi, button] = ginput(1);
		if ~isequal(button,1) % if enter is pressed
			break;
		else
			FIRST = true;
		end
	end
	
	print_title("Acquired obstacles",param.title_flags);

	
	% Set the punctual obstacles
	sgtitle("Select points to create punctual obstacles")
	idx = 1;
	x = [];
	y = [];
	while true
		[xi, yi ,button] = ginput(1);
		x = [x xi];
		y = [y yi];
		if ~isequal(button,1) % if enter is pressed
			break;
		end
		if rand() < param.percentage_static_obstacles
			obstacles{idx} = OBSTACLE(xi,yi, false, param);
		else
			obstacles{idx} = OBSTACLE(xi,yi, true, param);
		end
		obstacles{idx}.plot();
		idx = idx + 1;
	end
	print_title("Acquired punctual obstacles",param.title_flags);
	
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
	x = [];
	y = [];
	% Set the robots
	sgtitle("Select points to create robots")
	
	while true
		[xi, yi ,button] = ginput(1);
		if ~isequal(button,1) % if enter is pressed
			break;
		end
		x = [x xi];
		y = [y yi];
		plot(x, y, '+b');
		idx = idx + 1;
		if idx == param.N_MAX + 1
			break;
		end
	end

	param.N = idx - 1;
	for i = 1:param.N
		if rand() < param.percentage_linear_dynamics
			robots{i} = ROBOT([x(i);y(i)], i, 'linear', param);
		else
			robots{i} = ROBOT([x(i);y(i);rand()*2*pi], i, 'unicycle', param);
		end
		robots{i}.plot_real(all_markers, color_matrix, true,'on');
	end

	print_title("Acquired robots positions", param.title_flags);
	
	close(fig_1);
end
