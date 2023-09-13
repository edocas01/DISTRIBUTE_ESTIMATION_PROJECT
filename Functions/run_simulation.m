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
    figure()
    config;
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
            
            [u(:,i), barycenter(:,i)] = compute_control(R,T,R{i},parameters_simulation);

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

        %% PLOT

        clf
		hold on; grid on; 
        axis equal
		xlim([-40 40]); ylim([-40 40]);
		datas = data;
		datas.T.plot()
		plot(datas.circle_target(1,:), datas.circle_target(2,:),'b--', 'LineWidth', 1.5);
		for i = 1:parameters_simulation.N
			datas.R{i}.plot_real(all_markers, color_matrix, false);
			plot(datas.R{i}.voronoi);
			plot(datas.barycenter(1,i), datas.barycenter(2,i),'kx', 'LineWidth', 1);
		end
    	drawnow

        %% PLOT

        % Move the target
        T.dynamics(u_traj(:,t));

        if mod(t,round(Tmax/4)) == 0
            fprintf("Percentage of simulation: %d%%\n",round(t/Tmax*100))
            pause(0.5)
        end
    end


end