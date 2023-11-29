% D
% show the localization of a robot with its covariances
% extract a non linear robot for the dynamic symulation
X = []; % state of the state (x,y,theta) (N*3)
X_est = []; % estimated state of the state (x,y,theta) (N*3)
P = []; % std of the state (x,y,theta) (N*3)
ERR = []; % error of the state (x,y,theta) (N*3)

% find the first non linear robot
for i = 1:length(results{1}.R)
	if results{1}.R{i}.type == "unicycle"
		index = i;
		break;
	end
end

for i = 1:length(results)
	% unicycle robot
	robot = results{i}.R{index};
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

time = 0:1:length(results)-1;

% SELF LOCALIZATION 
fig = figure(10); 
% set(gcf, 'Position', get(0, 'Screensize'));
tiledlayout(3,1,'TileSpacing','compact', 'Padding','compact');

nexttile; hold on; grid on;
box on; 
plot(time, ERR(:,1), '-or','DisplayName','Error x');
plot(time, ERR(:,2), '-ob','DisplayName','Error y');
title(['Localization error x and y of robot ', num2str(index)], 'Interpreter', 'latex')
ylabel('Err. [m]','Interpreter', 'latex');
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
xlim([0, length(results)]);
ylim([min(ERR(:,1)), max(ERR(:,1))]);
set(gca,'xtick',[])

nexttile([1,1]); hold on; grid on;
box on; 
plot(time, ERR(:,3),'-ob','DisplayName','Error $\theta$');
title('Localization error $\theta$', 'Interpreter', 'latex')
ylabel('Err. [rad]', 'Interpreter', 'latex');
legend('Location','northwest', 'Interpreter', 'latex');
xlim([0, length(results)]);
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
xlim([0, length(results)]);

ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'b';


%%
% define the metrics for the equidistance and for the distance w.r.t. the target
values = [];
metrics = compute_metrics(results, parameters_simulation);
col_names = [];
for j = 1:length(metrics)
	err_dist = metrics{j}.err_dist;
	err_angle = metrics{j}.err_angles;
	mean_err_dist = mean(err_dist(err_dist < 3*parameters_simulation.DISTANCE_TARGET));
	std_err_dist = std(err_dist(err_dist < 3*parameters_simulation.DISTANCE_TARGET));
	mean_err_angle = mean(err_angle(err_angle < 100));
	std_err_angle = std(err_angle(err_angle < 100));
	values(1,j) = mean_err_dist;
	values(2,j) = std_err_dist;
	values(3,j) = mean_err_angle;
	values(4,j) = std_err_angle;
	if results{1}.R{j}.type == "unicycle"
		dyn = ' NL';
	else
		dyn = ' L';
	end
	col_names = [col_names,strjoin(["R", num2str(j), dyn],"")];
end


disp("Simulation metrics:")
T = table();
for j = 1:length(metrics)
	T = [T, table(values(:,j), 'VariableNames', col_names(j), 'RowNames', {'Mean Err. Distance', 'Std Err. Distance', 'Mean Err. Angle', 'Std Err. Angle'})];
end
disp(T);



% %%
% ERROR ON THE DISTANCE FROM TARGET
fig = figure(11);
% set(gcf, 'Position', get(0, 'Screensize'));
tiledlayout(2,1,'TileSpacing','compact', 'Padding','compact');

nexttile; hold on; grid on;
box on;
time = 0:1:length(metrics{1}.err_dist)-1;
for i = 1:length(metrics)
	if results{1}.R{i}.type == "unicycle"
		dyn = ' NL';
	else
		dyn = ' L';
	end
	tmp = metrics{i}.err_dist;
	tmp(tmp > 3*parameters_simulation.DISTANCE_TARGET) = NaN;
	plot(tmp, 'DisplayName', ['R.' num2str(i) dyn],'Color',color_matrix(i,:));
end
title('Distance on target error', 'Interpreter', 'latex'); 
ylabel('Error [m]', 'Interpreter', 'latex');
set(gca,'xtick',[])

xlim([0, length(metrics{1}.err_dist)-1]);
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');

% ERROR ON EQUIDISTANCE
nexttile([1,1]); hold on; grid on;
box on;
for i = 1:length(metrics)
	if results{1}.R{i}.type == "unicycle"
		dyn = ' NL';
	else
		dyn = ' L';
	end
	tmp = metrics{i}.err_angles;
	tmp(tmp == 100) = NaN;
	plot(time, tmp, 'DisplayName', ['R.' num2str(i) dyn],'Color',color_matrix(i,:));
end
title('Equidistance angle error', 'Interpreter', 'latex'); 
ylabel('Error [rad]', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylim("padded");
xlim([0, length(metrics{1}.err_dist)-1]);
legend('Location','northwest', 'Interpreter', 'latex', 'Orientation','horizontal');
