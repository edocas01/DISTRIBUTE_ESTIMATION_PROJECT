classdef ROBOT
	properties
		x           % x location
        y           % y location
        ComRadius   % Communication radius
	end
	methods 
        % Define the position
        function obj = setPosition(obj, valx, valy)
			obj.x = valx;
			obj.y = valy;
        end
        
        % Define communication radius
        function obj = setComRadius(obj, val_radius)
			obj.ComRadius = val_radius;
		end
			
	end
	
end
