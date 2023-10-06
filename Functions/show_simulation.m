% This function is used to show the simulation results
function show_simulation(results)
	config;
	figure(100);
	for t = 1:length(results)
		clf
		hold on; grid on; 
        axis equal
		xlim([-40 40]); ylim([-40 40]);
		datas = results;
		% datas = results{t};
		datas.T.plot()
		plot(datas.circle_target(1,:), datas.circle_target(2,:),'b--', 'LineWidth', 1.5);
		for i = 1:length(datas.R)
			datas.R{i}.plot_real(all_markers, color_matrix, true);
			plot(datas.R{i}.voronoi);
			plot(datas.barycenter(1,i), datas.barycenter(2,i),'kx', 'LineWidth', 1);
		end

		for i = 1:length(datas.O)
            plot(datas.O(:,1),datas.O(:,2),'sk','HandleVisibility', 'off','MarkerSize', 10,'LineWidth', 1.8);
		end

		for i = 1:length(datas.LO)
			datas.LO{i}.plot();
		end
    	drawnow
	end
end