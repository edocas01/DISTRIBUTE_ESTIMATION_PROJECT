function R = select_shape(N_robots, dyn_type, shape, center_point, distance, randdistance, param)
	R = cell(1, N_robots);
	if (~isvector(dyn_type)) || (length(dyn_type) ~= N_robots)
		error('Insert a dimension %d vector of dynamics type', N_robots)
	end
	switch shape
		case 'circle'
			for i = 1:N_robots
				if randdistance
					radius = distance + (-1 + 2 * rand()) * distance / 4;
				else
					radius = distance;
				end
				x = center_point(1) + radius * cos(2 * pi * i / N_robots);
				y = center_point(2) + radius * sin(2 * pi * i / N_robots);
				R{i} = ROBOT([x, y], i, dyn_type(i), param);
			end			
		case 'square'
			for i = 1:N_robots
				diagonal = distance * sqrt(2);
				if randdistance
					fr = distance / 8;
					A = center_point + [ diagonal ,  diagonal] + fr * (-1 + 2 * rand(1, 2)); 
					B = center_point + [-diagonal ,  diagonal] + fr * (-1 + 2 * rand(1, 2));
					C = center_point + [-diagonal , -diagonal] + fr * (-1 + 2 * rand(1, 2));
					D = center_point + [ diagonal , -diagonal] + fr * (-1 + 2 * rand(1, 2));
				else
					A = center_point + [ diagonal ,  diagonal]; 
					B = center_point + [-diagonal ,  diagonal];
					C = center_point + [-diagonal , -diagonal];
					D = center_point + [ diagonal , -diagonal];
				end
				% square vertex            % B----K----F----J----A
				vertex = [A; B; C; D];     % |                   |
				% Mid points               % L				     I
				E = (A + D) / 2;           % |                   |       
				F = (A + B) / 2;           % G                   E   
				G = (B + C) / 2;           % |                   |    
				H = (C + D) / 2;           % M					 |    
				midpoints = [E; F; G; H];  % |                   |         
				% Mid mid points           % C----N----H----O----D    
				I = (E + A) / 2;                    
				J = (F + A) / 2;                
				K = (B + F) / 2;
				L = (B + G) / 2;
				M = (C + G) / 2;
				N = (C + H) / 2;
				O = (D + H) / 2;
				midmidpoints = [I; J; K; L; M; N; O];
				if i <= 4
					R{i} = ROBOT(vertex(i, :), i, dyn_type(i), param);
				elseif i <= 8
					R{i} = ROBOT(midpoints(i - 4, :), i, dyn_type(i), param);
				elseif i <= 15
					R{i} = ROBOT(midmidpoints(i - 8, :), i, dyn_type(i), param);
				end
				% R{i} = ROBOT([x, y], i, dyn_type(i), param);
			end
		case 'triangle'
			% do something
		otherwise
			error('Insert a proper shape')
	end
		