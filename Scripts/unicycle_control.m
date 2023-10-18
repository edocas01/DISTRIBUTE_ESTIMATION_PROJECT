clear; close all; clc;
rng default;
config;
parameters_simulation.N = 1;
initial = [0;0;0];
R = ROBOT(initial,1,'unicycle',parameters_simulation);

dt = parameters_simulation.dt;

% define a circular polyshape
[x,y] = Circle(0, 0, R.ComRadius/2);

R.voronoi = polyshape(x,y);
quadratox = [1 1 10 10];
quadratoy = [-6 10 10 -6];
R.voronoi = subtract(R.voronoi,polyshape(quadratox,quadratoy));

% genera una serie di punti nel piano in un raggio di 10 metri
% anche con coordinate negative
X_ff = 10*rand(5,2) - 5;

for j = 1:size(X_ff,1)
	R.x = initial(1:2);
	R.th = initial(3);
	figure(j)
	hold on
	axis equal
	grid on
	% plot(R.voronoi)
	R.plot_real(all_markers, color_matrix, false);

	X_f = X_ff(j,:); 
	plot(X_f(1),X_f(2),'*r')
	% movimentare il robot
	for i = 1:10
	[dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation);
		R.dynamics([dx; dtheta]);
		R.plot_real(all_markers, color_matrix, false);
	end
end

function [dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation)
	kp = 1/dt;
	flag = 1;
	alpha = wrapTo2Pi(atan2(X_f(2)-R.x(2),X_f(1)-R.x(1)));
	gamma = alpha - R.th;
	cos_gamma = cos(gamma);

	v = kp*(norm(X_f - R.x));
	v = min(v,parameters_simulation.MAX_LINEAR_VELOCITY);
	dx = v*dt*cos_gamma;

	% if cos_gamma < 0
	% 	angle_final = angle_final + pi;
	% 	gamma = wrapTo2Pi(angle_final) - R.th;
	% end
	if alpha >= R.th 
		if gamma > pi
			gamma = 2*pi - gamma;
			flag = -1;
		else
			flag = 1;
		end
	else
		gamma = R.th - alpha;
		if gamma > pi
			gamma = 2*pi - gamma;
			flag = 1;
		else
			flag = -1;
		end
		
	end

	omega = kp*(gamma);
	omega = min(abs(omega),parameters_simulation.MAX_ANGULAR_VELOCITY);
	dtheta = omega*dt*flag;
end