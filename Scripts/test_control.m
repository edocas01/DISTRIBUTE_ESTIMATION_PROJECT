clc;
close all;
clearvars;
rng default;
config;
% [T, trajectory, u_trajectory, obstacles] = initialize_env(parameters_simulation);



[T,~,u_traj,~] = initialize_env(parameters_simulation);
fprintf("Target initial position: (%.2f m, %.2f m)\n", T.x(1), T.x(2));
coverage = 3;
N = parameters_simulation.N;
range = 10;

dyn_type = repmat("linear",N,1);
R = select_shape(N, dyn_type, 'circle', [0;0], range, 0, parameters_simulation);

  figure(1); clf
T.plot();
hold on; grid on; axis equal;
for i = 1:N
	R{i}.plot_real(all_markers, color_matrix, true);
end
hold off

pause(1)
%% Animation
tic

for i = 1:100
    for j = 1:length(R)
        EKF(R{j},0);
    end
    relative_general_consensous(R, T, parameters_simulation);
end

% IF A ROBOT HAS NO INFORMATIONS IT MOVES RANDOMLY

kp = 1 / parameters_simulation.dt;
for t = 1:length(u_traj(1,:))
	figure(2); clf
	xlim([-20 20])
	ylim([-20 20])
	hold on; grid on; axis equal;
    [circx, circy] = Circle(T.x(1), T.x(2), parameters_simulation.DISTANCE_TARGET);
    plot(circx, circy, '--', 'HandleVisibility','off','LineWidth',1.4)
	h = zeros(1,N+1);

    relative_general_consensous(R, T, parameters_simulation);
	title(sprintf("Time: %.2f s", t))
    voronoi_map_consensous(parameters_simulation, R, [], coverage);

	for i = 1:N
        
		[u(:,i), barycenter] = compute_control(R{i},parameters_simulation); 
       

		h(i) = R{i}.plot_real(all_markers, color_matrix, false);
		plot(R{i}.voronoi, 'HandleVisibility', 'off')
        if i > 1
            pippo = intersect(R{i}.voronoi,R{i-1}.voronoi);
            if pippo.NumRegions > 0
                error("Aoooo")
            end
        end
        if i == N
			h(i+1) = plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off','DisplayName','Centroid');
		else
			plot(barycenter(1), barycenter(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off')
		end
		
   	
    
        EKF(R{i}, u(:,i));
    end
    T.plot();
	legend show
	% legend(h, 'Location', 'bestoutside')
    T.dynamics(u_traj(:,t));
%     T.dynamics([0;0]);
    pause(0.01)
end
% hold off



% for i = 1:N
	
% 	pdemesh(msh);
% end


toc