% Font configuration
% set(0,'defaulttextinterpreter','latex')
% set(groot, 'defaultAxesTickLabelInterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');
% 
% set(0,'DefaultFigureWindowStyle','docked');
% set(0,'defaultAxesFontSize',  16)
% set(0,'DefaultLegendFontSize',16)


parameters_simulation = struct( ...
	... % Simulation parameters
	'dt', 							0.1,    		... % time step
	'size_map', 					50,  			... % size of the map
	'N_MAX', 						15,  			... % maximum number of agents
	... % Debug flags 	
	'title_flags', 					false, 			... % to print the central strings
	... % Robot parameters 	
	'MIN_RADIUS', 					0.8, 			... % minimum radius of the communication range (m)
	'MAX_RADIUS', 					2, 				... % maximum radius of the communication range (m)
	'std_gps', 						1, 				... % standard deviation of the GPS (m)
	'std_robots_model', 			1, 				... % standard deviation of the robot model (m)
	'std_relative_sensor', 			1,      		... % standard deviation of the relative sensor (m)
	... % Consensous parameters	
	'MSG_PROTOCOL', 				50				... % protocol used for message passing
	);

all_markers = {'o', 's', 'd', '*', '+', 'v', 'x', 'p', '^', '>', '<', 'h', '.', '_', '|'};
