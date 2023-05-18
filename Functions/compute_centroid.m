% given a density function phi, compute the barycenter of the cell
function [barycenter, msh] = compute_centroid(ps, phi)
	% from matlab documentation: https://it.mathworks.com/help/pde/ug/2-d-geometry-from-polyshape.html
	tr = triangulation(ps);
	model = createpde;
	tnodes = tr.Points';
	telements = tr.ConnectivityList';
	geometryFromMesh(model,tnodes,telements);
	msh = generateMesh(model, "Hmax", 0.5, "GeometricOrder","linear");
	[~, ai] = area(msh); % ai are the infinitesimal areas
	mass = 0;
	barycenter = zeros(2,1);
	% compute recursively the barycenter
	for i = 1:length(ai) 
		nodes = msh.Nodes(:,msh.Elements(:,i)); % nodes of the i-th element
		ci = mean(nodes,2); % centroid of the i-th element
		phi_i = phi(ci(1),ci(2)); % value of phi at the centroid
		mass = mass + ai(i) * phi_i; % mass of the cell
		barycenter = barycenter + ai(i) * phi_i * ci;
	end
	barycenter = barycenter / mass; % normalize the barycenter

end