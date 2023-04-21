function R = select_shape(N_robots, type, shape, center_point, distance, randdistance, param)
	R = cell(1, N_robots);
	if (~isvector(type)) || (length(type) ~= N_robots)
		error('Insert a dimension %d vector of dynamics type', N_robots)
	end
	% 27 degrees in radians

	switch shape
		case 'circle'
			for i = 1:N_robots
				if randdistance
					distance = distance + rand() * distance / 32;
				end
				x = center_point(1) + distance * cos(2 * pi * i / N_robots);
				y = center_point(2) + distance * sin(2 * pi * i / N_robots);
				R{i} = ROBOT([x, y], i, type(i), param);
			end			
		case 'square' % place the robots in the vertices of a square
			for i = 1:N_robots
				if randdistance
					distance = distance + rand() * distance / 32;
				end
				if i <= 4
					x = center_point(1) + distance * cos(pi / 4 + pi / 2 * (i - 1));
					y = center_point(2) + distance * sin(pi / 4 + pi / 2 * (i - 1));
				elseif i <= 8
					x = center_point(1) + (distance * sin(pi / 4)) * cos(pi / 2 + pi / 2 * (i - 1));
					y = center_point(2) + (distance * sin(pi / 4)) * sin(pi / 2 + pi / 2 * (i - 1));
				elseif i <= 10
					x = center_point(1) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * cos((pi / 2 - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 9) );
					y = center_point(2) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * sin((pi / 2 - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 9) );
				elseif i <= 12
					x = center_point(1) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * cos((pi - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 11));
					y = center_point(2) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * sin((pi - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 11));
				elseif i <= 14
					x = center_point(1) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * cos((3 * pi / 2 - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 13));
					y = center_point(2) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * sin((3 * pi / 2 - atan(1 / 2)) + 2 * atan(1 / 2) * (i - 13));
				elseif i <= 15
					x = center_point(1) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * cos((-atan(1 / 2)) + 0*2 * atan(1 / 2) * (i - 14));
					y = center_point(2) + (distance * sin(pi / 4) / cos(atan(1 / 2))) * sin((-atan(1 / 2)) + 0*2 * atan(1 / 2) * (i - 14));
				end
				R{i} = ROBOT([x, y], i, type(i), param);
			end
		case 'triangle'
			% do something
		otherwise
			error('Insert a proper shape')
	end
		