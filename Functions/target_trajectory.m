% This function is used to generate the target trajectory
function u_trajectory = target_trajectory(param,obstacles)
    u_trajectory = [];
	figure(1);
    xlim([-10,10]);
	if nargin > 1
		% Plot the obstacles
	end
	[x,y] = ginput;
	close figure 1;
	print_title("Acquired points");

	n = length(x);

	% inputs for the target
	for i = 1:n-1
		x_in = [x(i),y(i)];
		x_fin = [x(i+1), y(i+1)];
		% compute distance from the point to the next point
		dist = norm(x_fin - x_in);
		N_steps = round(dist/param.dt);
		% compute the velocity
		for j = 1:N_steps
			u_trajectory = [u_trajectory; (x_fin - x_in)/N_steps];
        end
    end
end