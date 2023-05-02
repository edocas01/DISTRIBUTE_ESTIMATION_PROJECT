% Font configuration
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
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
	'MIN_RADIUS', 					1, 			... % minimum radius of the communication range (m)
	'MAX_RADIUS', 					10, 				... % maximum radius of the communication range (m)
	'std_gps', 						1, 				... % standard deviation of the GPS (m)
	'std_robots_model', 			1, 				... % standard deviation of the robot model (m)
	'std_relative_sensor', 			0.3,      		... % standard deviation of the relative sensor (m)
	'MAX_VOLUME',					0.5, 			... % maximum radius of the robot volume (m)
	'MIN_VOLUME',					0.1, 			... % minimum radius of the robot volume (m)
	... % Consensous parameters	
	'MSG_PROTOCOL', 				50				... % protocol used for message passing
	);

% Define the colors in a matrix 15 by 3 (RGB) without red
color_matrix = [ ...
				[0, 0, 1]; 			... % blue
				[0, 1, 0]; 			... % green
				[0, 1, 1]; 			... % cyan
				[1, 0, 1]; 			... % magenta
				[1, 0.5, 0]; 		... % orange
				[0, 0, 0.5]; 		... % dark blue
				[0, 0.5, 0]; 		... % dark green
				[0, 0.5, 0.5]; 		... % dark cyan
				[0.5, 0, 0]; 		... % dark red
				[0.5, 0, 0.5]; 		... % dark magenta
				[0.5, 0.5, 0]; 		... % dark yellow
				[0.5, 0.5, 0.5]; 	... % dark gray
				[0.5, 0.25, 0]; 	... % dark brown
				[1, 0.75, 0.5]; 	... % light orange
				[0.75, 0.75, 0.75]; ... % light gray
				];


all_markers = {'o', 's', 'd', '*', '+', 'v', 'x', 'p', '^', '>', '<', 'h', '.', '_', '|'};
