function [xunit, yunit] = Circle(x, y, r, col, flag)
	hold on
	delta = 50;
	th = 0:pi/delta:2*pi-pi/delta;
	xunit = r * cos(th) + x;
	yunit = r * sin(th) + y;
end
	