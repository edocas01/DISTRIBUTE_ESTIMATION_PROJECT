function [xunit, yunit] = Circle(x, y, r)
	delta = 100;
	th = 0:2*pi/delta:2*pi;
	xunit = r * cos(th) + x;
	yunit = r * sin(th) + y;

	xunit = [xunit xunit(1)];
	yunit = [yunit yunit(1)];
end
	