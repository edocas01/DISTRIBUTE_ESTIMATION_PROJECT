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
%         P_est(k+1) = A(k) * P_est(k) * A(k)' + G(k) * Q(k) * G(k)'
% 
%         where:
%         A(k) is the jacobian of fk(x,u,nu) w.r.t the state e and evaluated in the x_est(k) and u(k) with Q = 0
%         G(k) is the jacobian of fk(x,u,nu) w.r.t Q e and evaluated in the x_est(k) and u(k) with Q = 0
%      
%      - update:
%         S(k+1) = H(k+1) * P_est(k+1) * H(k+1)' + R(k+1)
%         W(k+1) = P_est(k+1) * H(k+1)' / S(k+1)
%         x_est(k+1) = x_est(k+1) + W(k+1) (z(k+1) - hk+1(x_est(k+1)))
%         P_est(k+1) = (I - W(k+1) * H(k+1)) * P_est(k+1)
% 
%         where:
%         H(k+1) is the jacobian of hk+1(x,epsilon) w.r.t the state e and evaluated in the x_est(k+1) and epsilon = 0

	properties
		x; 				% real position of the robot

		x_est; 			% estimated state
		P;       		% covariance matrix of the state

		Z_gps; 			% GPS state measurement
		R_gps; 			% GPS measurement covariance matrix
		Q; 				% Uncertainty matrix of the model

		ComRadius; 		% communication radius
		target_est; 		% estimated target position (absolute)

		id; 				% id of the robot
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
    function obj = ROBOT(x, y, comradius, id)
		obj.x = zeros(2,1); 
		obj.x(1) = x;
		obj.x(2) = y;
		obj.x_est = obj.x;
		obj.P = eye(2);
		obj.ComRadius = comradius;	
		obj.R_gps = (rand(2,2) - 0.5);
		obj.R_gps = obj.R_gps * obj.R_gps';
		
		obj.Q = (rand(2,2) - 0.5);
		obj.Q = obj.Q * obj.Q';
		
		obj.Z_gps = zeros(2,1);	
		obj.target_est = zeros(2,1);
		obj.id = id;
			
    end
    
	% Update the position of the robot
	function obj = dynamics(obj, u)
		% linear dynamics with noise
		obj.x_est = obj.x_est + u + mvnrnd([0 0]', obj.Q);

		% linear dynamics without noise used in the gps measurement
		obj.x = obj.x + u;
	end


%{

 
   ____       _            _         __  __                _                   
  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
 

%}

	% function output = compute_control()
		
	% end

	function obj = GPS_measurement(obj)
		obj.Z_gps = obj.x + mvnrnd([0 0]', obj.R_gps)';
	end

	function plot(obj)
		plot(obj.x(1), obj.x(2), 'ok', 'DisplayName', ['robot ', num2str(obj.id)]);
		hold on;
		Circle(obj.x(1), obj.x(2), obj.ComRadius, '--k', false);
	end

			
	end % methods
	
end % classdef