classdef OBSTACLE < matlab.mixin.Copyable

%{

 
      _   _   _        _ _           _            
     / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
   / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
  /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
 

%}

	properties
		x 					% position of the obstacle
		vmax				% maximum velocity of the obstacle
		count_random_step 	% counter for the random step
		random_direction  	% random direction
	end

%{

 
   ____        _     _ _        __  __                _                   
  |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                          
 

%}

	methods 
		% Iniatialization of the obstacle
    	function obj = OBSTACLE(x, y, mobility, param)
			obj.x = [x;y];
			if mobility
				obj.vmax = param.vmax_obstacle;
			else
				obj.vmax = 0;
			end
			obj.count_random_step = 0;
			obj.random_direction = [];
    	end
    
		% Update the position of the obstacle
		function obj = dynamics(obj, param)
			u = [0;0];
			if obj.vmax > 0
				if obj.count_random_step == 0 || obj.count_random_step > 10 
					obj.count_random_step = 0;
					th = 2*pi*rand();
					obj.random_direction = [cos(th), sin(th)];
				end
				obj.count_random_step = obj.count_random_step + 1;
				u = obj.vmax*param.dt*obj.random_direction;
			end
			obj.x = obj.x + [u(1);u(2)];
			if abs(obj.x(1)) > param.size_map 
				obj.x(1) = sign(obj.x(1))*param.size_map;
				obj.count_random_step = 0;
			end
			if abs(obj.x(2)) > param.size_map 
				obj.x(2) = sign(obj.x(2))*param.size_map;
				obj.count_random_step = 0;
			end
			 
		end

%{

 
   ____       _            _         __  __                _                   
  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
 

%}
        function plot(obj)
			if obj.vmax > 0
				plot(obj.x(1), obj.x(2),'sb','HandleVisibility', 'off','MarkerSize', 10,'LineWidth', 2);
			else
		    	plot(obj.x(1), obj.x(2),'sk','HandleVisibility', 'off','MarkerSize', 10,'LineWidth', 2);
			end
	    end
			
	end % methods
	
end % classdef
