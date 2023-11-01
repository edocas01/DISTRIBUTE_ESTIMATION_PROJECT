clear; clc; close all;
settings_scripts;
config;
if ~exist('Results/results_for_metrics.mat', 'file')
	load('Setup/variables_for_metrics.mat');
	for i = 1:length(R)
		xR(i) = R{i}.x(1);
		yR(i) = R{i}.x(2);
	end
	
	clear R;
	R = cell(1, 2); % {{5 linear}, {5 linear and unicyle}}
	parameters_simulation.N = 5;
	parameters_simulation.CRASH_PERCENTAGE = 0;
	for i = 1:2
		for j = 1:length(xR)
			if (j <= 2  && i~=1) || i == 1 
				tmp{j} = ROBOT([xR(j);yR(j)], j, 'linear', parameters_simulation);
				tmp1{j} = ROBOT([xR(j);yR(j)], j, 'linear', parameters_simulation);
			else
				tmp_angle = rand()*2*pi;
				tmp{j} = ROBOT([xR(j);yR(j);tmp_angle], j, 'unicycle', parameters_simulation);
				tmp1{j} = ROBOT([xR(j);yR(j);tmp_angle], j, 'unicycle', parameters_simulation);
			end
		end
		R{i} = tmp;
		R{i+2} = tmp1;
	end

	tmp = T.x;

	for i = 1:2 % linear or unicycle
		results{i} = run_simulation(R{i}, T, [],[], u_traj, parameters_simulation); % Dynamic
		T.x = tmp;
		if i > 1
			show_simulation(results{i});
		end
	end
	save('Results/results_for_metrics.mat', 'results');
else
	load('Results/results_for_metrics.mat');
end
% results = [dinlin, dinun]


% show the localization of a robot with its covariances
% extract a non linear robot for the dynamic symulation
X = []; % state of the state (x,y,theta) (N*3)
X_est = []; % estimated state of the state (x,y,theta) (N*3)
P = []; % std of the state (x,y,theta) (N*3)
ERR = []; % error of the state (x,y,theta) (N*3)
for i = 1:length(results{1})
	% unicycle robot
	robot = results{2}{i}.R{5};
	X = [X; [robot.x' robot.th]];
	X_est = [X_est; [robot.x_est' robot.th_est]];
	P = [P; [sqrt(robot.P(1,1)) sqrt(robot.P(2,2)) sqrt(robot.P(3,3))]];
	tmp = robot.th-robot.th_est;
	if tmp > pi 
		tmp = 2*pi - tmp;
	elseif tmp < -pi
		tmp = 2*pi + tmp;
	end

	ERR = [ERR; [robot.x'-robot.x_est' tmp]];
end

time = 0:1:length(results{1})-1;
fig = figure(1); 
set(gcf, 'Position', get(0, 'Screensize'));
tiledlayout(3,1,'TileSpacing','compact', 'Padding','compact');

nexttile; hold on; grid on;
box on; 
plot(time, ERR(:,1), '-or','DisplayName','Error x');
plot(time, ERR(:,2), '-ob','DisplayName','Error y');
title('Localization error x and y', 'Interpreter', 'latex')
ylabel('Err. [m]','Interpreter', 'latex');
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
xlim([0, length(results{1})]);
ylim([min(ERR(:,1)), max(ERR(:,1))]);
set(gca,'xtick',[])

nexttile([1,1]); hold on; grid on;
box on; 
plot(time, ERR(:,3),'-ob','DisplayName','Error $\theta$');
title('Localization error $\theta$', 'Interpreter', 'latex')
ylabel('Err. [rad]', 'Interpreter', 'latex');
legend('Location','northwest', 'Interpreter', 'latex');
xlim([0, length(results{1})]);
ylim([min(ERR(:,3)), max(ERR(:,3))]);
set(gca,'xtick',[])

nexttile; hold on; grid on;
box on; 
yyaxis left
ylabel('Std [m]', 'Interpreter', 'latex');
plot(time, P(:,1), '-or','DisplayName','Std x');
plot(time, P(:,2), '-ok','DisplayName','Std y');

yyaxis right
plot(time, P(:,3), '-ob','DisplayName','Std $\theta$');
ylabel('Std [rad]', 'Interpreter', 'latex');
xlabel('Time [s]','Interpreter', 'latex');
title('Standard deviation of the localization', 'Interpreter', 'latex')
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
xlim([0, length(results{1})]);

ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'b';

saveas(fig,'IMAGES/ROBOT_LOCALIZATION/robot_localization.png');
saveas(fig,'IMAGES/ROBOT_LOCALIZATION/robot_localization.fig');



%%
% TODO define the metrics for the equidistance and for the distance w.r.t. the target
values = [];
for i = 1:length(results)
	metrics = compute_metrics(results{i}, parameters_simulation);
	if i == 2
		metrics_for_plot = metrics;
	end
	for j = 1:length(metrics)
		err_dist = metrics{j}.err_dist;
		err_angle = metrics{j}.err_angles;
		mean_err_dist = mean(err_dist);
		std_err_dist = std(err_dist);
		mean_err_angle = mean(err_angle(err_angle < 100));
		std_err_angle = std(err_angle(err_angle < 100));
		values(i*4-3,j) = mean_err_dist;
		values(i*4-2,j) = std_err_dist;
		values(i*4-1,j) = mean_err_angle;
		values(i*4,j) = std_err_angle;
		% create a latex file for the table
		create_macro_latex("latex_macros.tex",strjoin(["meandist", num2str(i*4-3), num2str(j)],""),values(i*4-3,j),'a');
		create_macro_latex("latex_macros.tex",strjoin(["stddist", num2str(i*4-2), num2str(j)],""),values(i*4-2,j),'a');
		create_macro_latex("latex_macros.tex",strjoin(["meanangle", num2str(i*4-1), num2str(j)],""),values(i*4-1,j),'a');
		create_macro_latex("latex_macros.tex",strjoin(["stdangle", num2str(i*4), num2str(j)],""),values(i*4,j),'a');
	end
end
% %%
fig = figure(2);
set(gcf, 'Position', get(0, 'Screensize'));
tiledlayout(2,1,'TileSpacing','compact', 'Padding','compact');

nexttile; hold on; grid on;
box on;
time = 0:1:length(metrics_for_plot{1}.err_dist)-1;
for i = 1:length(metrics_for_plot)
	if i == 4 || i == 5
		dyn = ' NL';
	else
		dyn = ' L';
	end
	plot(metrics_for_plot{i}.err_dist, 'DisplayName', ['R. ' num2str(i) dyn]);
end
title('Distance on target error', 'Interpreter', 'latex'); 
ylabel('Error [m]', 'Interpreter', 'latex');
set(gca,'xtick',[])

xlim([0, length(metrics_for_plot{1}.err_dist)-1]);
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');



nexttile([1,1]); hold on; grid on;
box on;
for i = 1:length(metrics_for_plot)
	if i == 4 || i == 5
		dyn = ' NL';
	else
		dyn = ' L';
	end
	tmp = metrics_for_plot{i}.err_angles;
	tmp(tmp == 100) = NaN;
	plot(time, tmp, 'DisplayName', ['R. ' num2str(i) dyn]);
end
title('Equidistance angle error', 'Interpreter', 'latex'); 
ylabel('Error [rad]', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylim("padded");
xlim([0, length(metrics_for_plot{1}.err_dist)-1]);
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');

saveas(fig,'IMAGES/SIMULATION_METRICS/simulation_metrics.png');
saveas(fig,'IMAGES/SIMULATION_METRICS/simulation_metrics.fig');


