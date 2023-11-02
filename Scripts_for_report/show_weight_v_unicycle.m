clearvars; clc; close all;

config;
settings_scripts;

dt = parameters_simulation.dt;

% define parameters
ic1 = [0;0;-pi/2];

X_f = [10;10];
Tmax = 50;

parameters_simulation.N = 1;

R1 = ROBOT(ic1,1,'unicycle',parameters_simulation);
R1.vmax(1) = 35 / 3.6;
R1.vmax(2) = pi/1.5;
R2 = ROBOT(ic1,2,'unicycle',parameters_simulation);
R2.vmax = R1.vmax;

X1 = [R1.x(1)];
Y1 = [R1.x(2)];
X2 = [R2.x(1)];
Y2 = [R2.x(2)];

for i = 1:Tmax
	u = generate_control(R1, X_f, dt, true);
    if norm(R1.x - X_f) > 0.1
	    R1.dynamics(u);
    end
	u = generate_control(R2, X_f, dt, false);
    if norm(R2.x - X_f) > 0.2
	    R2.dynamics(u);
    end
	X1 = [X1; R1.x(1)];
	Y1 = [Y1; R1.x(2)];
	X2 = [X2; R2.x(1)];
	Y2 = [Y2; R2.x(2)];
end

fig = figure(1)
fig.Color = [1 1 1];
set(fig, 'ToolBar', 'none');
set(fig,'Position', get(0, 'Screensize'));
subplot(1,2,1); hold on; grid on; axis equal; xlim([-1 11]); ylim([-1 11]);
xlabel('x [m]', 'Interpreter', 'latex'); ylabel('y [m]', 'Interpreter', 'latex'); title('Weighted velocity', 'Interpreter', 'latex');
plot(X1,Y1,'-*b','LineWidth', 2, 'HandleVisibility','off');
R1.plot_real(all_markers, color_matrix, false, 'off');
plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5, 'DisplayName','Goal');
legend('Location','northwest', 'Interpreter', 'latex');

subplot(1,2,2); hold on; grid on; axis square; xlim([0 17]-3.5); ylim([0 17] - 6);
xlabel('x [m]', 'Interpreter', 'latex'); ylabel('y [m]', 'Interpreter', 'latex'); title('Unweighted velocity', 'Interpreter', 'latex');
R2.plot_real(all_markers, color_matrix, false, 'off');
plot(X2,Y2,'-*k','LineWidth', 2, 'HandleVisibility','off');
plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5, 'DisplayName','Goal');
legend('Location','northwest', 'Interpreter', 'latex');


export_fig(fig,'IMAGES/TRAJECTORY_PLANNING/velocity_weight_unicycle.png');
saveas(fig,'IMAGES/TRAJECTORY_PLANNING/velocity_weight_unicycles.fig');

function u = generate_control(R, X_f, dt, weight_v)
	kp = 1/dt;
	flag = 1;
	alpha = wrapTo2Pi(atan2(X_f(2)-R.x(2),X_f(1)-R.x(1)));
	gamma = alpha - R.th;
	cos_gamma = cos(gamma);

	v = min(kp * (norm(X_f - R.x)), R.vmax(1));
	if weight_v
		v = v * cos_gamma;
	end
	dx = v*dt;
	
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
	omega = min(abs(omega),R.vmax(2));
	dtheta = omega*dt*flag;
	u = [dx;dtheta];
end