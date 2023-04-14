parameters_simulation = struct( ...
	'dt', 0.1, ...   % time step
	'tmax', 36, ...    % simulation time
	'N_MAX', 15, ...  % maximum number of agents
    'type_robot', 'linear' ... % define types of robot (if all equal robots are used)
);

all_markers = {'o', 's', 'd', '*', '+', 'v', 'x', 'p', '^', '>', '<', 'h', '.', '_', '|'};