% This function is used to generate the target trajectory
function u_trajectory = target_trajectory(param,obstacles)
    u_trajectory = [];
	figure(1);
    xlim([-10,10]);
	if nargin > 1
		% Plot the obstacles and use them as boundaries
	end
	[x,y] = ginput;
	close figure 1;
	print_title("Acquired points");

	n = length(x);

	% inputs for the target
	for i = 1:n-1
		% define the initial and final points
		x_in = [x(i),y(i)];
		x_fin = [x(i+1), y(i+1)];
		% compute distance from the point to the next point
		dist = norm(x_fin - x_in);
		% compute the number of steps as the distance divided by the time step
		N_steps = round(dist/param.dt);
		% compute the velocity
		for j = 1:N_steps
			% the iput is equal to the velocity for a certain number of steps
			u_trajectory = [u_trajectory; (x_fin - x_in)/N_steps];
        end
    end
end