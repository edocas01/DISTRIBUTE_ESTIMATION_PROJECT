% This function is used to show the simulation results
function show_simulation(results)
	config;
	figure();
	for t = 1:length(results)
		clf
		hold on; grid on; 
        axis equal
		xlim([-40 40]); ylim([-40 40]);
		datas = results{t};
		datas.T.plot()
		plot(datas.circle_target(1,:), datas.circle_target(2,:), 'b--', 'LineWidth', 1.5);
		for i = 1:parameters_simulation.N
			datas.R{i}.plot_real(all_markers, color_matrix, false);
			plot(datas.R{i}.voronoi);
			plot(datas.barycenter(1,i), datas.barycenter(2,i), 'kx', 'LineWidth', 1);
		end
    	drawnow
	end
end