classdef TARGET < handle

%{

 
      _   _   _        _ _           _            
     / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
   / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
  /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
 

%}

	properties
		x 				% real position of the target
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
    	function obj = TARGET(x,y)
			obj.x = [x;y];
    	end
    
		% Update the position of the robot
		function obj = dynamics(obj, dx,dy)
			% Simulate movement of the target
			obj.x = obj.x + [dx;dy];
		end

%{

 
   ____       _            _         __  __                _                   
  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
 

%}
        function plot(obj)
		    plot(obj.x(1), obj.x(2),'or','DisplayName','target');
		    hold on;
	    end
			
	end % methods
	
end % classdef
