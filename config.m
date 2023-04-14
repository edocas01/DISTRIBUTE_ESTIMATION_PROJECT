% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',  16)
set(0,'DefaultLegendFontSize',16)


parameters_simulation = struct( ...
	'dt', 0.1, ...   % time step
	'tmax', 36, ...    % simulation time
	'N_MAX', 15 ...  % maximum number of agents 
);

all_markers = {'o', 's', 'd', '*', '+', 'v', 'x', 'p', '^', '>', '<', 'h', '.', '_', '|'};