function barycenter = compute_centroid(ps, phi)
	tr = triangulation(ps);
	model = createpde;
	tnodes = tr.Points';
	telements = tr.ConnectivityList';
	geometryFromMesh(model,tnodes,telements);
	mesh = generateMesh(model, "Hmax", 0.5, "GeometricOrder","linear");
	[~, ai] = area(mesh);
	% ai are the infinitesimal areas
	mass = 0;
	barycenter = zeros(2,1);
	for i = 1:length(ai)
		nodes = mesh.Nodes(:,mesh.Elements(:,i));
		ci = mean(nodes,2);
		phi_i = phi(ci(1),ci(2));
		mass = mass + ai(i) * phi_i;
		barycenter = barycenter + ai(i) * phi_i * ci;
	end
	barycenter = barycenter / mass;

end