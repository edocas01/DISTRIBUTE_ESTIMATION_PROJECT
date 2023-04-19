classdef ROBOT < handle

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
		R_gps; 				% GPS measurement covariance matrix
		Q; 					% Uncertainty matrix of the dynamics model
		
		R_dist,             % distance measurement covariance matrix (1x1)
		target_est; 		% estimated target position (absolute)
		target_P;			% covariance matrix of the target position

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
    function obj = ROBOT(x, comradius, id, type)
		obj.x = zeros(2,1); 
		obj.x(1) = x(1);
		obj.x(2) = x(2);
		obj.x_est = obj.x;
		obj.P = eye(2);
		obj.ComRadius = comradius;	
		obj.R_gps = (rand(2,2) - 0.5) * 1;	% 1 m is the standard deviation of the gps measurement
		obj.R_gps = obj.R_gps * obj.R_gps';
		
		obj.Q = (rand(2,2) - 0.5);
		obj.Q = obj.Q * obj.Q';
		
		obj.R_dist = randn()^2;			% 0.2 m is the standard deviation of the distance measurement
		obj.target_est = zeros(2,1);
		obj.target_P = eye(2);
		obj.id = id;

		obj.type = type;
        
    end

	     
	% Update the position of the robot
	function obj = dynamics(obj, u)
		if obj.type == 'linear'		
			% linear dynamics with noise
			obj.x_est = obj.x_est + u + mvnrnd([0;0], obj.Q)';

			% linear dynamics without noise used in the gps measurement
			obj.x = obj.x + u;
		end
	end
	


	% Jacobian of the state function
	function J_X = jacobian_state(obj)
		if obj.type == 'linear'
			J_X = eye(2);
		end
	end

	% Jacobian of the noise function
	function J_Q = jacobian_noise(obj)
		if obj.type == 'linear'
			J_Q = eye(2);
		end
	end
	
	% Jacobian of the measurement function
	function J_H = jacobian_measurement(obj)
		if obj.type == 'linear'
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
	function plot(obj, all_markers)
		plot(obj.x_est(1), obj.x_est(2), strcat(all_markers{obj.id},'b'), 'DisplayName', ['robot ', num2str(obj.id)]);
		hold on;
		Circle(obj.x_est(1), obj.x_est(2), obj.ComRadius, '--b', false);
	end

			
	end % methods
	
end % classdef
