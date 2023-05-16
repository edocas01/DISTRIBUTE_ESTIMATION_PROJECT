% clc; clear; close all;

v1 = [0;0];
v2 = [10;0];
v3 = [10,3];
v4 = [0,3];
tic
pp = polyshape([v1(1),v2(1),v3(1),v4(1)],[v1(2),v2(2),v3(2),v4(2)]);
pp = robots{1}.voronoi;
phi = @(x,y) x.^2 + y.^2; % kg / m^2

centroid = compute_centroid(pp, phi);
% % From documentation:
% tr = triangulation(pp);
% model = createpde;
% tnodes = tr.Points';
% telements = tr.ConnectivityList';
% geometryFromMesh(model,tnodes,telements);
% mesh = generateMesh(model, "Hmax", 0.5, "GeometricOrder","linear");
% [~, ai] = area(mesh);
% % ai are the infinitesimal areas
% mass = 0;
% barycenter = zeros(2,1);
% for i = 1:length(ai)
% 	nodes = mesh.Nodes(:,mesh.Elements(:,i));
% 	ci = mean(nodes,2);
% 	phi_i = phi(ci(1),ci(2));
% 	mass = mass + ai(i) * phi_i;
% 	barycenter = barycenter + ai(i) * phi_i * ci;
% end
% barycenter = barycenter / mass;


% generateMesh creates a fine discretization of the given polyshape.
% GeometricOrder has to be set to linear
% Hmax -> is the maximum discretization step
% Hmin -> is the minimum discretization step
% centroid = 0;

toc


% xmin = min(pp.Vertices(:,1));
% xmax = max(pp.Vertices(:,1));
% ymin = min(pp.Vertices(:,2));
% ymax = max(pp.Vertices(:,2));

% inside = @(x,y) double(inpolygon(x, y, pp.Vertices(:,1), pp.Vertices(:,2)));
% func = @(x,y) phi(x,y) .* inside(x,y);
% mass_01 = integral2(func, xmin, xmax, ymin, ymax);

% xc = integral2(@(x,y) func(x,y) .* x, xmin, xmax, ymin, ymax) / mass_01;
% yc = integral2(@(x,y) func(x,y) .* y, xmin, xmax, ymin, ymax) / mass_01;