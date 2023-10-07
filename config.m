% Font configuration
set(0,'defaulttextinterpreter','none')
set(groot, 'defaultAxesTickLabelInterpreter','none');
set(groot, 'defaultLegendInterpreter','none');
% 
% set(0,'DefaultFigureWindowStyle','docked');
% set(0,'defaultAxesFontSize',  16)
% set(0,'DefaultLegendFontSize',16)


parameters_simulation = struct( ...
	... % Simulation parameters
	'dt', 							0.1,    		... % time step
	'size_map', 					50,  			... % size of the map
	'N_MAX', 						15,  			... % maximum number of agents
	'DEBUG', 						false,          ... % Debug flags
	... % Debug flags 	
	'title_flags', 					false, 			... % to print the central strings
	... % Robot parameters 	
	'MIN_Rc', 						7, 				... % minimum radius of the communication range (m)
	'MAX_Rc', 						15, 			... % maximum radius of the communication range (m)
	'std_gps', 						1, 				... % standard deviation of the GPS (m)
	'std_robots_model', 			1, 				... % standard deviation of the robot model (m)
	'std_relative_sensor', 			0.3,      		... % standard deviation of the relative sensor (m)
	'MAX_VOLUME',					0.2, 			... % maximum radius of the robot volume (m)
	'MIN_VOLUME',					0.4, 			... % minimum radius of the robot volume (m)
	'MAX_LINEAR_VELOCITY', 			50/3.6, 		... % maximum linear velocity (m/s)
	'MIN_LINEAR_VELOCITY', 			20/3.6, 		... % minimum linear velocity (m/s)
	'MAX_ANGULAR_VELOCITY', 		pi/2, 			... % maximum angular velocity (rad/s)
	'MIN_ANGULAR_VELOCITY', 		pi/6, 			... % minimum angular velocity (rad/s)
	... % Target parameters
	'vmax_target', 					10/3.6,		 	... % velocity of target (m/s)
	... % Punctual obstacles parameters
	'vmax_obstacle', 				10/3.6, 			... % velocity of the obstacle (m/s)
	'percentage_static_obstacles',  0.8, 			... % percentage of static obstacles
	... % Consensous parameters	
	'MSG_PROTOCOL', 				50,				... % protocol used for message passing
	... % Voronoi parameters
	'coverage', 					3, 				... % coverage for the uncertainties 
	... % Control parameters
	'DISTANCE_TARGET',				3, 				... % distance from the target (m)
	'TOLERANCE_DISTANCE', 			0 				... % tolerance distance on the circle (m)
	);
	parameters_simulation.TOLERANCE_DISTANCE = parameters_simulation.DISTANCE_TARGET*0.5;

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
