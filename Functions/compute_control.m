% This function compute the inputs for the robots according to the controls
function [u, barycenter] = compute_control(robot,param)
	%% control on the circle around the target
	radius = param.DISTANCE_TARGET;
	func = @(x,y,r,x_t,y_t) exp(-r/10*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2); % KEEP the "4"
	phi = @(x,y) func(x, y, radius, robot.all_robots_pos(end-1), robot.all_robots_pos(end));
	[barycenter, msh] = compute_centroid(robot, phi, radius);
	

	% other controls ...

	kp = 1/param.dt;
	%% compute the control
	if  kp * norm(barycenter - robot.x_est) < robot.vmax
		u = kp * (barycenter - robot.x_est) * param.dt;
	else
		u = robot.vmax * param.dt * (barycenter - robot.x_est) / norm(barycenter - robot.x_est);
	end
end