function show_ellipses_consensus(R,flag)
	config;
	figure(324);
	hold on;
	axis equal;

	for i = 1:size(R.all_robots_pos,1)/2
		x = R.all_robots_pos(2*i-1);
		y = R.all_robots_pos(2*i);
		point_ellipse = compute_ellipse([x;y], R.all_cov_pos(2*i-1:2*i,2*i-1:2*i),3);
		if i < size(R.all_robots_pos,1)/2
			c = color_matrix(i,:);
		else 
			c = [1,0,0];
		end
		if flag 
			plot(point_ellipse(1,:), point_ellipse(2,:), 'Color', c);
		else
			plot(point_ellipse(1,:), point_ellipse(2,:), '--', 'Color', c);
		end
	end
		
end