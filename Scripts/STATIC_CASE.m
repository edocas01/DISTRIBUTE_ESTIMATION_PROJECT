%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Classes
% classdef ROBOT < handle
% %{

%       _   _   _        _ _           _            
%      / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%     / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%    / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
%   /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
% %}

% properties
% 	x	 					% x position
%   ComRadius   % Communication radius
% end

% %{
%    ____        _     _ _        __  __                _                  
%   |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
%   | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
%   |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \
%   |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                        
% %}
% methods 
%     % This function will be called only 1 time, then the position will be controlled by the internal dynamics
%     function obj = Initialize(val_x, val_y, val_ComRadius)
%         obj.x(1) = val_x;
%         obj.x(2) = val_y;
%         obj.ComRadius = val_ComRadius;
%     end





% 		end



% %{
%    ____       _            _         __  __                _                   
%   |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
%   | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
%   |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
%   |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
% %}
	
% end % methods
	
% end % classdef

% classdef TARGET < handle
% %{

%       _   _   _        _ _           _            
%      / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
%     / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
%    / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
%   /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
% %}

% properties
% 	x	 					% x position
%   ComRadius   % Communication radius
% end

% %{
%    ____        _     _ _        __  __                _                  
%   |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
%   | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
%   |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \
%   |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                        
% %}
% methods 
%     % This function will be called only 1 time, then the position will be controlled by the internal dynamics
%     function obj = Initialize(val_x, val_y, val_ComRadius)
%         obj.x(1) = val_x;
%         obj.x(2) = val_y;
%         obj.ComRadius = val_ComRadius;
%     end





% 		end



% %{
%    ____       _            _         __  __                _                   
%   |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
%   | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
%   |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
%   |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
% %}
	
% end % methods
	
% end % classdef



clc
close all
clearvars

addpath("Classes\")
addpath("Functions\")



%% Entities definition

% Target definition
x_T = 0;
y_T = 0;
ComRadius_T = 1;

T = TARGET().Initialize(x_T, y_T, ComRadius_T);

% Robots
n_robot = 6;
xR_min = -5;
xR_max = 5;
yR_min = -5;
yR_max = 5;
ComRadius_R = 1 + rand(1, n_robot); % Comunication radii [1 - 2] 

for i = 1 : n_robot
	R.(strcat("R",num2str(i))) = ROBOT();
	R.(strcat("R",num2str(i))) = R.(strcat("R",num2str(i))).setPosition(xR_min + (xR_max - xR_min) * rand(), yR_min + (yR_max - yR_min) * rand());
	R.(strcat("R",num2str(i))) = R.(strcat("R",num2str(i))).setComRadius(ComRadius_R(i));
end

% Plotting
figure(1)
ax = gca;
% set(gca,'FontSize',14)
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
axis equal
hold on
grid on

plot(T.x(1), T.x(2), 'r*')
for i = 1:n_robots
	plot(Robots{i}.x(1), Robots{i}.x(2), 'b*')
	Circle(Robots{i}.x(1), Robots{i}.x(2), Robots{i}.ComRadius)
end
hold off


%% Localization with kalman filter


