function mass = compute_centroid(polyshape, phi)
	pp = polyshape;
	xmin = min(pp.Vertices(:,1));
	xmax = max(pp.Vertices(:,1));
	ymin = min(pp.Vertices(:,2));
	ymax = max(pp.Vertices(:,2));

	inside = @(x,y) double(inpolygon(x, y, pp.Vertices(:,1), pp.Vertices(:,2)));
	func = @(x,y) phi(x,y) .* inside(x,y);
	mass = integral2(func, xmin, xmax, ymin, ymax);
end