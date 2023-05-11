function [pts, rx, ry, th] = ellipse(c, P)
	[eigenvec, eigenval] = eig(P);

	[rx, idx] = max(diag(eigenval));
	rx = sqrt(rx);
	rx_dir = eigenvec(:,idx);

	ry = min(eigenval(1,1), eigenval(2,2));
	ry = sqrt(ry);

	th = atan2(rx_dir(2), rx_dir(1));

	npts = 30;
	t = linspace(0, 2 * pi, npts);
	Rth = [cos(th) -sin(th); sin(th) cos(th)];
	c = [c(1); c(2)];
	pts = zeros(2, npts);
	for i = 1:npts
		pts(:,i) = c + Rth * [rx * cos(t(i)); ry * sin(t(i))];
	end
end