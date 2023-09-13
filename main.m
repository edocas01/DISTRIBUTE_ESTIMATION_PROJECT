%% Syntax functions
% Robot_cell_array = select_shape(N_robots, type_dynamics, shape, center_point, distance, randdistance, param)

clc;
clear;
close all;
clearvars;
rng default;

addpath('Scripts');
addpath('Functions');
addpath('Classes');
addpath('Results');
name = input("Save the results: ", "s");
% ------------------------ %
%  DEFINE DEFAULT SETTINGS %
%  ----------------------- %
config;
parameters_simulation;


clc;
[T,~,u_traj,O,R] = initialize_env(parameters_simulation);
fprintf("Target initial position: (%.2f m, %.2f m)\n", T.x(1), T.x(2));
N = parameters_simulation.N;
range = 10;

% dyn_type = repmat("linear",N,1);
% R = select_shape(N, dyn_type, 'circle', [0;0], range, 0, parameters_simulation);

figure(1); clf
T.plot();
hold on; grid on; axis equal;

for i = 1:N
	R{i}.plot_real(all_markers, color_matrix, true);
end

for i = 1:length(O)
    O{i}.plot();
end

hold off
legend('Location','eastoutside')
pause(1)
%% Calculations	
tic
results = run_simulation(R, T, O, u_traj, parameters_simulation);
if (~isempty(name))
    name = ['Results/',name,'.mat'];
    save(name, "results");
end
toc

%% Animation
tic  
show_simulation(results)
toc