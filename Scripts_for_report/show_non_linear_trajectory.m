clear all;
close all;
clc;
settings_scripts;
config;
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'defaultAxesFontSize',  20)
set(0,'DefaultLegendFontSize', 20)
dt = parameters_simulation.dt;
% define parameters
ic1 = [0;0;2*pi/3];
ic2 = [-pi/2];
X_f = [10;10];
Tmax = 50;

% create the robot
parameters_simulation.N = 1;
R1 = ROBOT(ic1,1,'unicycle',parameters_simulation);
R1.vmax(1) = 50/3.6;
R1.vmax(2) = pi/1.5;
R2 = ROBOT([ic1(1:2);ic2],1,'unicycle',parameters_simulation);
R2.vmax = R1.vmax(1);
R2.vmax(2) = R1.vmax(2);
R3 = ROBOT(ic1(1:2), 1, 'linear', parameters_simulation);
R3.vmax(1) = R1.vmax(1); 
R4 = ROBOT([0;0;-pi/2],4,'unicycle',parameters_simulation);
R4.vmax(1) = 35 / 3.6;
R4.vmax(2) = pi/1.5;

X1 = [R1.x(1)];
Y1 = [R1.x(2)];
X2 = [R2.x(1)];
Y2 = [R2.x(2)];
X3 = [R3.x(1)];
Y3 = [R3.x(2)];
X4 = [R4.x(1)];
Y4 = [R4.x(2)];

% myVideo = VideoWriter('IMAGES/TRAJECTORY_PLANNING/trajectory_planning'); %open video file
% myVideo.FrameRate = 6;  
% open(myVideo)

% figure(1)
% set(gcf, 'Position', get(0, 'Screensize'));
% subplot(1,3,1); hold on; axis equal; grid on;
% R3.plot_real(all_markers, color_matrix, false); xlim([-2 11]); ylim([-1 11]); xlabel('x [m]'); ylabel('y [m]'); title('Linear dynamics');
% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);

% subplot(1,3,3); hold on; axis equal; grid on; xlim([-2 11]); ylim([-1 11]); xlabel('x [m]'); ylabel('y [m]'); title('Unicycle dynamics');
% R1.plot_real(all_markers, color_matrix, false);
% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);

% subplot(1,3,2); hold on; axis equal; grid on;
% R2.plot_real(all_markers, color_matrix, false); xlim([-2 11]); ylim([-1 11]); xlabel('x [m]'); ylabel('y [m]'); title('Unicycle dynamics');
% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);

for i = 1:Tmax
	u = generate_control(R1,X_f,dt);
    if norm(R1.x - X_f) > 0.1
	    R1.dynamics(u);
    end
	u = generate_control(R2,X_f,dt);
    if norm(R2.x - X_f) > 0.2
	    R2.dynamics(u);
    end
	
	kp = 1/dt;
	if  kp * norm(X_f - R3.x) < R3.vmax
		u = kp * (X_f - R3.x) * dt;
	else
		u = R3.vmax * dt * (X_f - R3.x) / norm(X_f - R3.x);
	end
	R3.dynamics(u);

	u = generate_control(R4,X_f,dt);
    if norm(R4.x - X_f) > 0.2
	    R4.dynamics(u);
    end

	% update the positions
	X1 = [X1,R1.x(1)];
	Y1 = [Y1,R1.x(2)];
	X2 = [X2,R2.x(1)];
	Y2 = [Y2,R2.x(2)];
	X3 = [X3,R3.x(1)];
	Y3 = [Y3,R3.x(2)];
	X4 = [X4,R4.x(1)];
	Y4 = [Y4,R4.x(2)];

	% clf; 
	% plot the positions
	% subplot(1,3,1); hold on; axis equal; grid on; xlim([-2 11]); ylim([-1 11]);xlabel('x [m]'); ylabel('y [m]'); title('Linear dynamics');
	% R3.plot_real(all_markers, color_matrix, false);
	% plot(X3,Y3,'-*r','LineWidth',2);
	% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);

	% subplot(1,3,3); hold on; axis equal; grid on; xlim([-2 11]); ylim([-1 11]);xlabel('x [m]'); ylabel('y [m]'); title('Unicycle dynamics');
	% R2.plot_real(all_markers, color_matrix, false);
	% plot(X2,Y2,'-*k','LineWidth',2);
	% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);
	
	% subplot(1,3,2); hold on; axis equal; grid on; xlim([-2 11]); ylim([-1 11]); xlabel('x [m]'); ylabel('y [m]'); title('Unicycle dynamics');
	% plot(X1,Y1,'-*b','LineWidth',2);
	% R1.plot_real(all_markers, color_matrix, false);
	% plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',3,'MarkerSize',5);
	
	
	
	% drawnow;
	% frame = getframe(gcf); %get frame
	% writeVideo(myVideo, frame);


end

% close(myVideo)

close all;
fig = figure(1);
set(gcf, 'Position', get(0, 'Screensize'));
subplot(1,2,1);hold on; box on;
axis equal; grid on; xlim([-6 11]); ylim([-6 11]); xlabel('x [m]'); ylabel('y [m]'); title('Linear and unicycle path');
plot(X1,Y1,'-*b','LineWidth',2,'DisplayName','Unicycle: (0,0,$\frac{2\pi}{3}$)');
plot(X2,Y2,'-*k','LineWidth',2,'DisplayName','Unicycle: (0,0,$-\frac{\pi}{2}$)');
plot(X3,Y3,'-*r','LineWidth',2,'DisplayName','Linear: (0,0)');
plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',4,'MarkerSize',10, 'DisplayName','Goal');
legend('Location','northwest');

subplot(1,2,2);hold on; box on;
axis square; grid on; xlim([0 17]-3.5); ylim([0 17] - 6); xlabel('x [m]'); ylabel('y [m]'); title('Unweighted velocity (not implemented)', 'Interpreter', 'latex');
plot(X4,Y4,'-*k','LineWidth',2,'DisplayName','Unicycle: (0,0,$-\frac{\pi}{2}$)');
plot(X_f(1),X_f(2),'o','Color',[0.9290 0.6940 0.1250],'LineWidth',4,'MarkerSize',10, 'DisplayName','Goal');
legend('Location','northwest');
% save the figure
export_fig(fig,'IMAGES/TRAJECTORY_PLANNING/trajectory_planning.png');
saveas(fig,'IMAGES/TRAJECTORY_PLANNING/trajectory_planning.fig');






% compute the control for non linear robots
function u = generate_control(R, X_f,dt)
	kp = 1/dt;
	flag = 1;
	alpha = wrapTo2Pi(atan2(X_f(2)-R.x(2),X_f(1)-R.x(1)));
	gamma = alpha - R.th;
	cos_gamma = cos(gamma);
	
	v = kp*(norm(X_f - R.x));
	v = min(v,R.vmax(1));
	if R.id ~= 4
		dx = v*dt*cos_gamma;
	else
		dx = v*dt;
	end
	
	percentage = 1;
	outside = true;
	
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