clear; close all; clc;
rng default;
config;
parameters_simulation.N = 1;

R = ROBOT([0,0,pi/5],1,'unicycle',parameters_simulation);

dt = parameters_simulation.dt;

% define a circular polyshape
[x,y] = Circle(0, 0, R.ComRadius/2);

R.voronoi = polyshape(x,y);
quadratox = [1 1 10 10];
quadratoy = [-6 10 10 -6];
R.voronoi = subtract(R.voronoi,polyshape(quadratox,quadratoy));

figure(1)
hold on
axis equal
grid on
plot(R.voronoi)
R.plot_real(all_markers, color_matrix, false);

X_f = [-4; 4];

[x,y,theta] = generate_trajectory(R, X_f, dt, parameters_simulation);
plot(X_f(1),X_f(2),'*r','LineWidth',2);
plot(x,y,'-*r','LineWidth',1.1);

% movimentare il robot
[dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation);
R.dynamics([dx; dtheta]);
R.plot_real(all_markers, color_matrix, false);



function [x,y,th] = generate_trajectory(R, X_f, dt, parameters_simulation)
		dt_new = 0:dt/100:dt;
		x = [R.x(1)];
		y = [R.x(2)];
		th = [R.th];
		[dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation);
		dx = dx/(length(dt_new)-1);
		dtheta = dtheta/(length(dt_new)-1);
		
		
		for i = 1:length(dt_new)-1
			R = [cos(th(end) + dtheta), 0;
				 sin(th(end) + dtheta), 0;
				 0, 1];
			tmp = [x(1);y(1);th(1)] + R*[dx*i;dtheta*i];
			x_new = tmp(1);
			y_new = tmp(2);
			th_new = wrapTo2Pi(tmp(3));
			x = [x; x_new];
			y = [y; y_new];
			th = [th; th_new];
		end
end

function [dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation)
	kp = 1/dt;
	v = kp*(norm(X_f - R.x));
	v = min(v,parameters_simulation.MAX_LINEAR_VELOCITY);
	dx = v*dt;
	angle = atan2(X_f(2)-R.x(2),X_f(1)-R.x(1));
	omega = kp*(wrapTo2Pi(angle) - R.th);
	omega = min(omega,parameters_simulation.MAX_ANGULAR_VELOCITY);
	dtheta = omega*dt;
end