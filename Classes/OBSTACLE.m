classdef OBSTACLE < handle

%{

 
      _   _   _        _ _           _            
     / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
   / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
  /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
 

%}

	properties
		x 				% position of the obstacle
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
    	function obj = OBSTACLE(x,y)
			obj.x = [x;y];
    	end
    
		% Update the position of the obstacle
		function obj = dynamics(obj, u)
			obj.x = obj.x + [u(1);u(2)];
		end

%{

 
   ____       _            _         __  __                _                   
  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
 

%}
        function plot(obj)
		    plot(obj.x(1), obj.x(2),'sk','HandleVisibility', 'off','MarkerSize', 10,'LineWidth', 2);
		    hold on;
	    end
			
	end % methods
	
end % classdef
