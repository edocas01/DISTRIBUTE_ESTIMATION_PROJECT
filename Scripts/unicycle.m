config;
close all;
parameters_simulation.N = 1; 
R = ROBOT([0,0,0],1,'unicycle',parameters_simulation);
dt = parameters_simulation.dt;

kp = 1/dt;

X_f = [30;70];
Tmax = 10;
T = 0:dt:Tmax;
X = [];
TH = [];
X_est = [];
TH_est = [];
figure(1)
for i = 1:length(T)
	if norm(X_f - R.x_est) > 1
		v = kp*(norm(X_f - R.x_est));
		v = min(v,parameters_simulation.MAX_LINEAR_VELOCITY);
		v = v*dt;
		angle = atan2(X_f(2)-R.x_est(2),X_f(1)-R.x_est(1));
		omega = kp*(angle - R.th_est);
		omega = min(omega,parameters_simulation.MAX_ANGULAR_VELOCITY);
		omega = omega*dt;
	else
		v = 0;
		omega = 0;
	end
	X_est = [X_est,R.x_est];
	TH_est = [TH_est,R.th_est];
	X = [X,R.x];
	TH = [TH,R.th];
	EKF(R, [v;omega]);
	
	clf;
	axis equal; grid on; hold on;
	R.plot_real(all_markers, color_matrix, true);
	drawnow
end

figure()
plot(X(1,:),X(2,:),'b')
hold on
plot(X_est(1,:),X_est(2,:),'r')
plot(X_f(1), X_f(2), 'om')
legend('real','estimated','final point')

figure()
plot(T,rad2deg(TH),'b')
hold on
plot(T,rad2deg(TH_est),'r')
legend('real','estimated')

