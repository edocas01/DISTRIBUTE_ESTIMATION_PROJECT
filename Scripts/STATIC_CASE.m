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

rng('default');

%% Entities definition

% Target definition
x_T = 0;
y_T = 0;
ComRadius_T = 1;

T = TARGET().Initialize(x_T, y_T, ComRadius_T);

% Robots definition
n_robots = 2;
ComRadius_vec = [5 5 5 5];
Robots = cell(n_robots,1);

% Robots initialisation
for i = 1:n_robots
	dist = 3 + rand(1)*2;
	% Generate a random angle with a range of pi/2 starting from 0, the from pi /2 to pi and so on
	th = pi/2 * (i - 1) + (i * pi/2 - pi/2 * (i - 1))  * rand(1);
	
	Robots{i} = ROBOT().Initialize(T.x(1) + dist*cos(th), T.x(2) + dist*sin(th), ComRadius_vec(i), i);
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


