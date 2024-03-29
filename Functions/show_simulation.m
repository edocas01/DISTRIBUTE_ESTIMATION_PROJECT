% This function is used to show the simulation results
function show_simulation(results)
	config;
    figure(5)
    size_map = parameters_simulation.size_map;
	for t = 1:length(results)
		if length(results) > 1
			clf
		end
		hold on; grid on; 
        axis equal
		xlim([-size_map size_map]); ylim([-size_map size_map]);
		datas = results{t};
		datas.T.plot()
		plot(datas.circle_target(1,:), datas.circle_target(2,:),'b--', 'LineWidth', 1.5);
		for i = 1:length(datas.R)
			if datas.R{i}.robot_crash == false
				datas.R{i}.plot_real(all_markers, color_matrix, true);
				plot(datas.R{i}.voronoi, 'HandleVisibility', 'off', 'FaceColor', color_matrix(datas.R{i}.id,:));
				%plot(datas.barycenter(1,i), datas.barycenter(2,i),'kx', 'LineWidth', 1, 'HandleVisibility', 'off');
			else
				plot(datas.R{i}.x(1),datas.R{i}.x(2),'kx', 'LineWidth', 2, 'HandleVisibility', 'off');
            end
		end
		xlabel('x [m]', 'Interpreter', 'latex'); ylabel('y [m]', 'Interpreter', 'latex');
		title('Simulation environment', 'Interpreter', 'latex');
% 		legend('Location', 'EastOutside');
		for i = 1:length(datas.O)
            datas.O{i}.plot();
		end

		for i = 1:length(datas.LO)
			datas.LO{i}.plot();
        end

		if length(results) > 1
			drawnow
		end
			
	end
end