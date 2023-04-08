classdef ROBOT < handle
%{

      _   _   _        _ _           _            
     / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
    / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
   / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
  /_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
                                                  
%}

  properties
    id          % robot id
    x	 					% x position
    ComRadius   % Communication radius
  end

%{
   ____        _     _ _        __  __                _                  
  |  _ \ _   _| |__ | (_) ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | | | | '_ \| | |/ __| | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |_| | |_) | | | (__  | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|    \__,_|_.__/|_|_|\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                        
%}
  methods 
      % This function will be called only 1 time, then the position will be controlled by the internal dynamics
      function obj = Initialize(obj, val_x, val_y, val_ComRadius, val_id)
          obj.x(1) = val_x;
          obj.x(2) = val_y;
          obj.ComRadius = val_ComRadius;
          obj.id = val_id;
      end

  end



%{
   ____       _            _         __  __                _                   
  |  _ \ _ __(_)_   ____ _| |_ ___  |  \/  | ___ _ __ ___ | |__   ___ _ __ ___ 
  | |_) | '__| \ \ / / _` | __/ _ \ | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
  |  __/| |  | |\ V / (_| | ||  __/ | |  | |  __/ | | | | | |_) |  __/ |  \__ \
  |_|   |_|  |_| \_/ \__,_|\__\___| |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
                                                                               
%}
		
end % classdef
