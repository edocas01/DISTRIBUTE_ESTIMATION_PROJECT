clear; clc; close all;
settings_scripts;
config;
if ~exist('Results/results_for_metrics.mat', 'file')
	load('Setup/variables_for_metrics.mat');

    
	R = cell(1, 4); % {{5 linear}, {5 linear and unicyle}, {5 linear}, {5 linear and unicyle}}
	parameters_simulation.N = 5;
	parameters_simulation.CRASH_PERCENTAGE = 0;
	xR = [25, 25, 25, 31, 31];
	yR = [-33, -31, -29, -32, -28];
	for i = 1:2
		for j = 1:length(xR)
			if (j <= 3  && i~=1) || i == 1 
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
		results{2*i-1} = run_simulation(R{i}, T, [],[], u_traj, parameters_simulation); % Dynamic (first or third simulation)
		T.x = tmp;
		if i > 1
			show_simulation(results{2*i-1});
		end
		results{2*i} = run_simulation(R{i+2}, T, [], [], u_traj.*0, parameters_simulation); % Static (second or fourth simulation)
		T.x = tmp;
		show_simulation(results{2*i});

	end
	save('Results/results_for_metrics.mat', 'results');
else
	load('Results/results_for_metrics.mat');
end
% results = [dinlin, statlin, dinun, statun]


% show the localization of a robot with its covariances
% extract a non linear robot for the dynamic symulation
X = []; % state of the state (x,y,theta) (N*3)
X_est = []; % estimated state of the state (x,y,theta) (N*3)
P = []; % std of the state (x,y,theta) (N*3)
ERR = []; % error of the state (x,y,theta) (N*3)
for i = 1:length(results{1})
	% unicycle robot
	robot = results{3}{i}.R{5};
	X = [X; [robot.x' robot.th]];
	X_est = [X_est; [robot.x_est' robot.th_est]];
	P = [P; [sqrt(robot.P(1,1)) sqrt(robot.P(2,2)) sqrt(robot.P(3,3))]];

	ERR = [ERR; [robot.x'-robot.x_est' robot.th-robot.th_est]];
end

time = 0:1:length(results{1})-1;
figure(1); hold on; grid on;
yyaxis left
plot(time, ERR(:,1), 'r','DisplayName','Error x');
plot(time, ERR(:,2), 'b','DisplayName','Error y');
ylabel('Error in x and y (m)');

yyaxis right
plot(time, ERR(:,3), 'g','DisplayName','Error theta');

xlabel('Time (s)');
ylabel('Error in $\theta$ (rad)');
legend('Location','best')


% TODO define the metrics for the equidistance and for the distance w.r.t. the target