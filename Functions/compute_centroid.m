% given a density function phi, compute the barycenter of the cell
% if the robot is outside the circle, it has to reach the target (objective i nan)
% if the robot is already in the circle, it has to move in the circumference
function [barycenter, msh] = compute_centroid(robot, phi, objective, param)
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

	% compute recursively the barycenter if phi ~= 0 (robot has to be still -> already in the circle
	% but without neighbors on it)
	if ~ (class(phi) == "function_handle") 
		barycenter = [NaN;NaN];
	else
		% if ci is further than the robot with respect to the desired circle (radius), its weight is less important
		% compute the intermedium point, the same if has to reach a defined point or a circle
		if isnan(objective(1))
			radius = param.DISTANCE_TARGET;
			objective = robot.all_robots_pos(end-1:end);
		else
			radius = 0;
			objective = [objective(1); objective(2)];
		end

		for i = 1:length(ai) 
			nodes = msh.Nodes(:,msh.Elements(:,i)); % nodes of the i-th element
			ci = mean(nodes,2); % centroid of the i-th element

			dir = robot.x_est - objective;
			dir = dir/norm(dir);
			intermedium = objective + dir*radius; 
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
	
	if isnan(barycenter(1)) || isnan(barycenter(2))
		barycenter = robot.x_est;
	end
end