% Font configuration
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

set(0,'DefaultFigureWindowStyle','docked');
set(0,'defaultAxesFontSize',  16)
set(0,'DefaultLegendFontSize',16)


parameters_simulation = struct( ...
	'dt', 			0.1,    		...% time step
	'size_map', 	50,  			...% size of the map
	'N_MAX', 		15,  			...% maximum number of agents 
	'MSG_PROTOCOL', 10				...% protocol used for message passing
);

all_markers = {'o', 's', 'd', '*', '+', 'v', 'x', 'p', '^', '>', '<', 'h', '.', '_', '|'};