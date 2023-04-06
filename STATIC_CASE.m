clc
close all
clearvars

addpath("Classes\")

%% Classes
% ROBOT class 
% classdef ROBOT
% 	properties
% 		x           % x location
%         y           % y location
%         ComRadius    % Communication radius
% 	end
% 	methods 
%         % Define the position
%         function obj = setPosition(obj, valx, valy)
% 			obj.x = valx;
% 			obj.y = valy;
%         end        
%         % Define communication radius
%         function obj = setComRadius(obj, val_radius)
% 			obj.ComRadius = val_radius;
% 		end			
% 	end	
% end

% TARGET class
% classdef TARGET
% 	properties
% 		  x           % x location
%         y           % y location
%         ComRadius    % Communication radius
% 	end
% 	methods 
%         % Define the position
%         function obj = setPosition(obj, valx, valy)
% 			obj.x = valx;
% 			obj.y = valy;
%         end       
%         % Define communication radius
%         function obj = setComRadius(obj, val_radius)
% 			obj.ComRadius = val_radius;
% 		end			
% 	end	
% end

%% Entities definition
% Target
x_T = 0;
y_T = 0;
ComRadius_T = 1;

T = TARGET();
T.setPosition(x_T, y_T);
T.setComRadius(ComRadius_T);

% Robots
n_robot = 6;
xR_min = -5;
xR_max = 5;
yR_min = -5;
yR_max = 5;
ComRadius_R = rand(1, n_robot); % Communication radius array 

for i = 1 : n_robot
	R.(strcat("R",num2str(i))) = ROBOT();
	R.(strcat("R",num2str(i))) = R.(strcat("R",num2str(i))).setPosition(xR_min + (xR_max - xR_min) * rand(), yR_min + (yR_max - yR_min) * rand());
	R.(strcat("R",num2str(i))).setComRadius(ComRadius_R(i));
end

%% Plot the entities
figure(1)
hold on
grid on
axis equal
axis([-6 6 -6 6])
xlabel("x")
ylabel("y")
title("Entities")
plot(T.x, T.y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
for i = 1 : n_robot
	plot(R.(strcat("R",num2str(i))).x, R.(strcat("R",num2str(i))).y, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b')
end






