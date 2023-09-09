% This function runs the simulation of the robots
function results = run_simulation(R, T, u_traj, parameters_simulation)
	Tmax = length(u_traj(1,:));
	results = cell(1,Tmax);
    
    % Initial localizations of the robots
    for i = 1:10
        for j = 1:length(R)
            EKF(R{j},0);
        end
        relative_general_consensous(R, T, parameters_simulation);
    end
    % Simulation
    u_uno = [];
    for t = 1:Tmax
        [circx, circy] = Circle(T.x(1), T.x(2), parameters_simulation.DISTANCE_TARGET);
        

        relative_general_consensous(R, T, parameters_simulation);
        voronoi_map_consensous(parameters_simulation, R, []);
        
        % Saving the results
        data.T = copy(T);
        for i = 1:length(R)
            R{i} = copy(R{i});
        end
        data.R = R;
		data.circle_target = [circx;circy];
		
        for i = 1:parameters_simulation.N
            
            [u(:,i), barycenter(:,i)] = compute_control(R{i},parameters_simulation);
            if i == 1
                u_uno(:,t) = u(:,i);
            end
            
            for j = 1:parameters_simulation.N
                inters = intersect(R{i}.voronoi,R{j}.voronoi);
                if inters.NumRegions > 0 && parameters_simulation.DEBUG && i~=j
                    warning("Intersection of cells")
                end
            end
                     
            EKF(R{i}, u(:,i));
        end
        
        data.barycenter = barycenter;
		results{t} = data;

        % Move the target
        T.dynamics(u_traj(:,t));

        if mod(t,round(Tmax/4)) == 0
            fprintf("Percentage of simulation: %d%%\n",round(t/Tmax*100))
            pause(0.5)
        end
    end


end