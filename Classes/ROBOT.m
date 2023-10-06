classdef ROBOT < matlab.mixin.Copyable

%{

 
      _   _   _        _ _           _            
     / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
   / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
  /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
 

%}

% The model of the analized problem is:
%     
%     x(k+1) = fk(x(k),u(k),Q(k)) -> Q is the noise in the model
%     z(k) = hk(x(k),epsilon(k)) -> epsilon is the noise on the measures
% 		
% The Ekf is divided into steps: 
%     - predicion:
%         x_est(k+1) = fk(x_est(k), u(k))
%         P_est(k+1) = J_X(k) * P_est(k) * J_X(k)' + J_Q(k) * Q(k) * J_Q(k)'
% 
%         where:
%         J_X(k) is the jacobian of fk(x,u,nu) w.r.t the state e and evaluated in the x_est(k) and u(k) with Q = 0
%         J_Q(k) is the jacobian of fk(x,u,nu) w.r.t Q e and evaluated in the x_est(k) and u(k) with Q = 0
%      
%      - update:
%         S(k+1) = H(k+1) * P_est(k+1) * H(k+1)' + R(k+1) 				% S(k+1) is the innovation covariance matrix
%         W(k+1) = P_est(k+1) * H(k+1)' / S(k+1)						% W(k+1) is the kalman gain
%         x_est(k+1) = x_est(k+1) + W(k+1) (z(k+1) - hk+1(x_est(k+1)))	% x_est(k+1) is the updated state
%         P_est(k+1) = (I - W(k+1) * H(k+1)) * P_est(k+1)				% P_est(k+1) is the updated covariance matrix
% 
%         where:
%         H(k+1) is the jacobian of hk+1(x,epsilon) w.r.t the state e and evaluated in the x_est(k+1) and epsilon = 0

	properties
		type; 				% type of robot (linear, unicycle, etc.)
		x_est; 				% estimated state
		P;       			% covariance matrix of the state
		ComRadius; 			% communication radius
		id; 				% id of the robot
		
		x; 					% real position of the robot
		x_in; 				% initial (real) position of the robot 
		R_gps; 				% GPS measurement covariance matrix
		Q; 					% Uncertainty matrix of the dynamics model
		
		H; 					% jacobian of the measurement model
		R_dist,             % distance measurement covariance matrix (1x1)
		target_est; 		% estimated target position (absolute)
		target_P;			% covariance matrix of the target position

		target_est_hist;    % history of the target estimation
		target_P_hist;		% history of the target covariance matrix

		neighbors; 			% list of the neighbors of the robot
		all_robots_pos; 	% list of the neighbors positions (also target)
		all_cov_pos; 		% covariance of the neighbors positions (also target)

		voronoi; 			% polyshape of the voronoi region of the robot
		volume; 			% volume occupied by the robot
		vmax; 				% maximum velocity of the robot

        set_distance_radius % for the control
		count_random_step   % for the control in case of random movement
		random_direction    % for the control in case of random movement

	end
%{

   ____        _     _ _        __  __                _                   
  |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                          
 

%} 

	methods 
	% Iniatialization of the robot
    function obj = ROBOT(x, id, type, param)

		obj.type = type;
		if strcmp(obj.type, 'linear')
			obj.x = zeros(2,1); 
			obj.x(1) = x(1);
			obj.x(2) = x(2);
			obj.x_in = obj.x; % Save initial position in case of initiailzation
			obj.x_est = obj.x;
			obj.P = eye(2);
			obj.Q = (rand(2,2) - 0.5) * param.std_relative_sensor;
			obj.Q = obj.Q * obj.Q';
			obj.vmax = rand() * (param.MAX_LINEAR_VELOCITY - param.MIN_LINEAR_VELOCITY) + param.MIN_LINEAR_VELOCITY;
		elseif strcmp(obj.type, 'unicycle')
			obj.x = zeros(3,1); 
			obj.x(1) = x(1);
			obj.x(2) = x(2);
			obj.x(3) = x(3);
			obj.x_in = obj.x;
			obj.x_est = obj.x;
			obj.P = eye(3);
			obj.Q = (rand(3,3) - 0.5) * param.std_relative_sensor;
			obj.Q = obj.Q * obj.Q';
			obj.vmax = [rand() * (param.MAX_LINEAR_VELOCITY - param.MIN_LINEAR_VELOCITY) + param.MIN_LINEAR_VELOCITY;...
					    rand() * (param.MAX_ANGULAR_VELOCITY - param.MIN_ANGULAR_VELOCITY) + param.MIN_ANGULAR_VELOCITY];
		end
			
		obj.ComRadius = rand()*(param.MAX_Rc - param.MIN_Rc) + param.MIN_Rc;
		obj.id = id;

		% Measurement model on the absolute position GPS
		obj.R_gps = (rand(2,2) - 0.5) * param.std_gps;	% 1 m is the standard deviation of the gps measurement
		obj.R_gps = obj.R_gps * obj.R_gps';
		
		% Measurement model on the relative position
		obj.H = eye(2);
		obj.R_dist = (rand(2,2) - 0.5) * param.std_relative_sensor;
		obj.R_dist = obj.R_dist * obj.R_dist';
		obj.target_est = zeros(2,1);
		obj.target_P = eye(2);
		obj.all_robots_pos = ones(2*(param.N+1), 1)*1e6;
		obj.all_cov_pos = eye(2*(param.N+1))*1e6;
		
		% To track the estimation after the consensus algorithm is completed
		obj.target_est_hist = [];
		obj.target_P_hist = {};

		% To compute voronoi
		obj.neighbors = ["init"];
		obj.voronoi = [];
		obj.volume = rand() * (param.MAX_VOLUME - param.MIN_VOLUME) + param.MIN_VOLUME;

        % To control if the robot is on the circle of the target
        obj.set_distance_radius = false;
		obj.count_random_step = 0;
		obj.random_direction = 0;
	end

	     
	% Update the position of the robot
	function obj = dynamics(obj, u)
		if strcmp(obj.type, 'linear')   
			% linear dynamics with noise
			obj.x_est = obj.x_est + u + mvnrnd([0;0], obj.Q)';
			% linear dynamics without noise used in the gps measurement
			obj.x = obj.x + u;
		elseif strcmp(obj.type, 'unicycle')
			% rotation matrix 3x3
			R = [cos(obj.x_est(3)), -sin(obj.x_est(3)), 0;
				 sin(obj.x_est(3)),  cos(obj.x_est(3)), 0;
				 0, 0, 1];
			% unicycle dynamics with noise
			obj.x_est = obj.x_est + R*u + mvnrnd([0;0;0], obj.Q)';
			% unicycle dynamics without noise used in the gps measurement
			obj.x = obj.x + R*u;
		end
	end
	
	% Jacobian of the state function
	function J_X = jacobian_state(obj)
		if strcmp(obj.type, 'linear')
			J_X = eye(2);
		elseif strcmp(obj.type, 'unicycle')
			J_X = [1, 0, -obj.x_est(2)*u(1) - obj.x_est(1)*u(2);
				   0, 1,  obj.x_est(1)*u(1) - obj.x_est(2)*u(2);
				   0, 0,  1];
		end
	end

	% Jacobian of the noise function
	function J_Q = jacobian_noise(obj)
		if strcmp(obj.type, 'linear')
			J_Q = eye(2);
		elseif strcmp(obj.type, 'unicycle')
			J_Q = eye(3);
		end
	end
	
	% Jacobian of the measurement function
	function J_H = jacobian_measurement(obj)
		if strcmp(obj.type, 'linear')
			J_H = eye(2);
		end
	end
	
	% Perform a gps measure considering the real position of the robot
	function Z_gps = GPS_measurement(obj)
		Z_gps = obj.x + mvnrnd([0;0], obj.R_gps)';
	end
	
	
	
	%{
	
	 
	   ____       _            _         __  __                _                   
	  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
	  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
	  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
	  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
																				   
	 
	
	%}

	% Plot the position of the robot with its communication radius
	% TODO:
	% - Add the possibility to plot the covariance ellipse
	function h = plot_est(obj, all_markers, color_matrix, plot_circle)
		h = plot(obj.x_est(1), obj.x_est(2), strcat(all_markers{obj.id},'b'), 'DisplayName', ['Robot ', num2str(obj.id)], 'MarkerSize', 10, 'Color', color_matrix(obj.id,:),'LineWidth', 1.5);
		hold on;
		[ptsx, ptsy] = Circle(obj.x_est(1), obj.x_est(2), obj.volume);
		vol = polyshape(ptsx, ptsy);
		plot(vol, 'HandleVisibility', 'off', 'FaceAlpha', 0.4, 'FaceColor', 'k');
		if plot_circle
			[x,y] = Circle(obj.x_est(1), obj.x_est(2), obj.ComRadius);
			plot(x,y, '--k', 'HandleVisibility', 'off');
		end
	end

	function h = plot_real(obj, all_markers, color_matrix, plot_circle)
		h = plot(obj.x(1), obj.x(2), strcat(all_markers{obj.id},'b'), 'DisplayName', ['Robot ', num2str(obj.id)], 'MarkerSize', 10, 'Color', color_matrix(obj.id,:),'LineWidth', 1.5);
		hold on;
		[ptsx, ptsy] = Circle(obj.x(1), obj.x(2), obj.volume);
		vol = polyshape(ptsx, ptsy);
		plot(vol, 'HandleVisibility', 'off', 'FaceAlpha', 0.4, 'FaceColor', 'k');
		if plot_circle
			[x,y] = Circle(obj.x(1), obj.x(2), obj.ComRadius);
			plot(x,y, '--k', 'HandleVisibility', 'off');
		end
	end

	function Initialize_Position(obj)
		obj.x = obj.x_in;
	end
	
	function Clear_Targ_Estimates(obj)
		obj.target_est = zeros(2,1);
		obj.target_P = eye(2);
	end

	function Clear_Targ_Estimates_Hist(obj)
		obj.target_est_hist = [];
		obj.target_P_hist = {};
		obj.target_est_hist_messages = [];
		obj.target_P_hist_messages = {};
	end
		
	end % methods
	
end % classdef