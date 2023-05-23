% given a density function phi, compute the barycenter of the cell
function [barycenter, msh] = compute_centroid(robot, phi, radius)
	% from matlab documentation: https://it.mathworks.com/help/pde/ug/2-d-geometry-from-polyshape.html
	tr = triangulation(robot.voronoi);
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
	
		% if ci is further than the robot with respect to the desired circle (radius), its weight is less important

		% compute the intermedium point
		dir = robot.x_est - robot.all_robots_pos(end-1:end);
		dir = dir/norm(dir);
		intermedium = robot.all_robots_pos(end-1:end) + dir*radius; 
		if norm(ci - intermedium) > norm(robot.x_est - intermedium)
			phi_i = phi(ci(1),ci(2))*0.1; % value of phi at the centroid (decremented)
		else
			phi_i = phi(ci(1),ci(2)); % value of phi at the centroid
		end

        if phi_i < eps
            warning("Phi is too small")
        end
		mass = mass + ai(i) * phi_i; % mass of the cell
		barycenter = barycenter + ai(i) * phi_i * ci;
	end
	barycenter = barycenter / mass; % normalize the barycenter
end