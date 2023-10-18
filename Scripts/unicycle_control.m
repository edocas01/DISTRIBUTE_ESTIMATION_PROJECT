clear; close all; clc;
rng default;
config;
parameters_simulation.N = 1;

R = ROBOT([0,0,pi/5],1,'unicycle',parameters_simulation);

dt = parameters_simulation.dt;

% define a circular polyshape
[x,y] = Circle(0, 0, R.ComRadius/2);

% R.voronoi = polyshape(x,y);
% quadratox = [1 1 10 10];
% quadratoy = [-6 10 10 -6];
% R.voronoi = subtract(R.voronoi,polyshape(quadratox,quadratoy));

Tmax = 5;
X_f1 = [-10; 10];
X_f2 = [10; 4];
X_f3 = [2; -3];

figure(1); hold on; axis equal; grid on;
R.plot_real(all_markers, color_matrix, false);
xlim([-12,12]);
ylim([-12,12]);
x = [R.x(1)];
y = [R.x(2)];
for i = dt:dt:Tmax
	[dx,dtheta] = generate_control(R, X_f1,dt);
	R.dynamics([dx;dtheta]);
	x = [x; R.x(1)];
	y = [y; R.x(2)];
	clf;
	hold on; axis equal; grid on;
	R.plot_real(all_markers, color_matrix, false);
	plot(x,y,'-*r');
	text(X_f1(1),X_f1(2),'1');
	text(X_f2(1),X_f2(2),'2');
	text(X_f3(1),X_f3(2),'3');
	xlim([-12,12]);
	ylim([-12,12]);
	drawnow
end
for i = dt:dt:Tmax
	[dx,dtheta] = generate_control(R, X_f2,dt);
	R.dynamics([dx;dtheta]);
	x = [x; R.x(1)];
	y = [y; R.x(2)];
	clf;
	hold on; axis equal; grid on;
	R.plot_real(all_markers, color_matrix, false);
	plot(x,y,'-*r');
	text(X_f1(1),X_f1(2),'1');
	text(X_f2(1),X_f2(2),'2');
	text(X_f3(1),X_f3(2),'3');
	xlim([-12,12]);
	ylim([-12,12]);
	drawnow
end
for i = dt:dt:Tmax
	[dx,dtheta] = generate_control(R, X_f3,dt);
	R.dynamics([dx;dtheta]);
	x = [x; R.x(1)];
	y = [y; R.x(2)];
	clf;
	hold on; axis equal; grid on;
	R.plot_real(all_markers, color_matrix, false);
	plot(x,y,'-*r');
	text(X_f1(1),X_f1(2),'1');
	text(X_f2(1),X_f2(2),'2');
	text(X_f3(1),X_f3(2),'3');
	xlim([-12,12]);
	ylim([-12,12]);
	drawnow
end

% function [x,y,th] = generate_trajectory(R, X_f, dt, parameters_simulation)
% 		dt_new = 0:dt/100:dt;
% 		x = [R.x(1)];
% 		y = [R.x(2)];
% 		th = [R.th];
% 		[dx,dtheta] = generate_control(R, X_f,dt,parameters_simulation);
% 		dx = dx/(length(dt_new)-1);
% 		dtheta = dtheta/(length(dt_new)-1);
% 		for i = 1:length(dt_new)-1
% 			R = [cos(th(end) + dtheta), 0;
% 				 sin(th(end) + dtheta), 0;
% 				 0, 1];
% 			tmp = [x(1);y(1);th(1)] + R*[dx*i;dtheta*i];
% 			x_new = tmp(1);
% 			y_new = tmp(2);
% 			th_new = wrapTo2Pi(tmp(3));
% 			x = [x; x_new];
% 			y = [y; y_new];
% 			th = [th; th_new];
% 		end
% end

function [dx,dtheta] = generate_control(R, X_f,dt)
	Xf_angle = atan2(X_f(2) - R.x(2), X_f(1) - R.x(1)) - R.th;
    rad2deg(Xf_angle)
	kp = 1/dt;
	if Xf_angle > pi/6 && Xf_angle < 2 * pi - pi / 6
		% go backwards
		v = -min(kp * norm(X_f - R.x), R.vmax(1));
		omega = min(kp * Xf_angle, R.vmax(2));
	else
		% go forward
		v = min(kp * norm(X_f - R.x), R.vmax(1));
		omega = min(kp * Xf_angle, R.vmax(2));
	end

	dx = v * dt;
	dtheta = omega * dt;
end

