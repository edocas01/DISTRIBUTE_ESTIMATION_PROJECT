clc; clear all; close all;
n=5;
x = 720*rand(1, n);
y = 360*rand(1, n);
voronoi(x,y,'--k');
DT = DelaunayTri(x', y');
% use this in newer versions of matlab
% DT = delaunayTriangulation (x, y);
hold all;
triplot(DT,'k-');
[centers, radii] = DT.circumcenters();
theta = -pi:pi/20:pi;
for iCircle=1:size(centers,1)
xCircle = centers(iCircle,1) + radii(iCircle)*cos(theta);
yCircle = centers(iCircle,2) + radii(iCircle)*sin(theta);
plot(xCircle, yCircle, 'b-');
end
plot(x,y,'+k', 'MarkerSize', 12)
axis equal;
hold on
