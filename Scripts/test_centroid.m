v1 = [0;0];
v2 = [10;0];
v3 = [10,3];
v4 = [0,3];
tic
pp = polyshape([v1(1),v2(1),v3(1),v4(1)],[v1(2),v2(2),v3(2),v4(2)]);
phi = @(x,y) 1;

mass = compute_centroid(robots{1}.voronoi,phi);
toc