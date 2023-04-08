%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
close all
clearvars

addpath("Classes\")
addpath("Functions\")

rng('default');

%% Entities definition
n_robots = 3;
% Robot cell array
Robots = cell(n_robots,1);

% Robots initialisation
for i = 1:n_robots
	Robots{i} = Robot();
end

