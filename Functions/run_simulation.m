% This function runs the simulation of the robots
function results = run_simulation(R, T, O, LO, u_traj, parameters_simulation)
    parameters_simulation.N = length(R);
	Tmax = length(u_traj(1,:));
	results = cell(1,Tmax);
    % Initial localizations of the robots
    for i = 1:10
        % for j = 1:length(R)
        %     EKF(R{j},[0;0]);
        % end
        relative_general_consensous(R, T, parameters_simulation);
    end
    % Simulation
    for t = 1:Tmax

        [circx, circy] = Circle(T.x(1), T.x(2), parameters_simulation.DISTANCE_TARGET);
        relative_general_consensous(R, T, parameters_simulation);
        voronoi_map_consensous(parameters_simulation, R, T, O, LO);
        
        % Saving the results
        data.u_traj = u_traj;
        data.T = copy(T);
        for i = 1:length(R)
            R{i} = copy(R{i});
        end
        data.R = R;
        
        for i = 1:length(O)
            O{i} = copy(O{i});
        end
        data.O = O;

        for i = 1:length(LO)
            data.LO{i} = LO{i};
        end
        data.LO = LO;

		data.circle_target = [circx;circy];
		
        for i = 1:parameters_simulation.N
            
            [u(:,i), barycenter(:,i)] = compute_control(R{i},parameters_simulation);

            for j = 1:parameters_simulation.N
                inters = intersect(R{i}.voronoi,R{j}.voronoi);
                if inters.NumRegions > 0 && parameters_simulation.DEBUG && i~=j
                    warning("Intersection of cells")
                end
            end
                     
            EKF(R{i}, u(:,i));

            if R{i}.robot_crash == false
                for k = 1:length(LO)
                    if inpolygon(R{i}.x(1),R{i}.x(2),LO{k}.poly.Vertices(:,1),LO{k}.poly.Vertices(:,2))
                        R{i}.robot_crash = true;
                        warning("Robot crashed");
                        break;
                    end
                end
            end
        end

        data.u = u;
        data.barycenter = barycenter;
		results{t} = data;

        % Move the target
        T.dynamics(u_traj(:,t));
        % Move the obstacles
        for i = 1:length(O)
            O{i}.dynamics(parameters_simulation);
        end

        if mod(t,round(Tmax/4)) == 0
            fprintf("Percentage of simulation: %d%%\n",round(t/Tmax*100))
            % destroy randomly a robot
            if rand() < parameters_simulation.CRASH_PERCENTAGE
                R{randi(length(R))}.robot_crash = true;
                disp("Robot destroyed on purpose");
            end
        end
    end



end