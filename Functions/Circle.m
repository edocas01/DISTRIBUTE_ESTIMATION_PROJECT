function h = Circle(x, y, r, col, flag)
	hold on
	th = 0:pi/50:2*pi;
	xunit = r * cos(th) + x;
	yunit = r * sin(th) + y;
	if flag == false
		plot(xunit, yunit, col, 'HandleVisibility','off');
	else
		plot(xunit, yunit, col);
	end
end
	