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
%         S(k+1) = J_H(k+1) * P_est(k+1) * J_H(k+1)' + R(k+1) 				% S(k+1) is the innovation covariance matrix
%         W(k+1) = P_est(k+1) * J_H(k+1)' / S(k+1)						% W(k+1) is the kalman gain
%         x_est(k+1) = x_est(k+1) + W(k+1) (z(k+1) - hk+1(x_est(k+1)))	% x_est(k+1) is the updated state
%         P_est(k+1) = (I - W(k+1) * J_H(k+1)) * P_est(k+1)				% P_est(k+1) is the updated covariance matrix
% 
%         where:
%         H(k+1) is the jacobian of hk+1(x,epsilon) w.r.t the state e and evaluated in the x_est(k+1) and epsilon = 0

function EKF(robot, u) % u is already the delta_x and delta_y
	J_X = robot.jacobian_state();
	J_Q = robot.jacobian_noise();
	J_H = robot.jacobian_measurement();
	% prediction step
	robot.dynamics(u);	
	robot.P = J_X * robot.P * J_X' + J_Q * robot.Q * J_Q';

	% update step
	S = J_H * robot.P * J_H' + robot.R_gps; 
	W = robot.P * J_H' / S;
	tmp = W * (robot.GPS_measurement() - robot.x_est(1:2));
	robot.x_est = robot.x_est + tmp(1:2);
	robot.th_est = wrapTo2Pi(robot.th_est + wrapTo2Pi(tmp(end)));
	robot.P = (eye(size(robot.P)) - W * J_H) * robot.P;
end