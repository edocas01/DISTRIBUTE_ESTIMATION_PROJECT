function [large_obstacles] = create_LO()
    set(gcf, 'Position', get(0, 'Screensize'));
get(0,'defaultfigureposition');


idx = 1;
FIRST = false;
first_time = true;
exit = false;
pl = {};
while true
	x = [];
	y = [];
	X = [];
	while true
		if ~FIRST
			[xi, yi ,button] = ginput(1);
		end

		if first_time
			if ~isequal(button,1) % if enter is pressed
				exit = true;
				break;
			end
		end
		first_time = false;

		if size(X,1) > 2
			if ~isequal(button,1) % if enter is pressed
				break;
			end
		end
		FIRST = false;
		x = [x xi];
		y = [y yi];
		X = [x' y'];
		pl{end+1} = plot(x, y, '--k','HandleVisibility','off');
	end 
	
	if exit
		break;
	end
	large_obstacles{idx} = LARGE_OBSTACLE(X);
	break;
end
for i = 1:length(pl)
	delete(pl{i});
end

end