clc;
close all;
clearvars;
rng default;
config;
parameters_simulation.N = 1;
% create the robot
radius_voronoi = 5;

R = ROBOT([2.78;1.45], 1, 'linear', parameters_simulation);
for i = 1:10
	EKF(R,0);
end
[cx,cy] = Circle(R.x_est(1),R.x_est(1),radius_voronoi);
poly_voronoi = polyshape(cx,cy);
R.voronoi = poly_voronoi;


% create the large obstacles
fig_1 = figure(1); clf;
% Set the large obstacles
sgtitle("Select points to create obstacles")
hold on;
grid on;
axis equal;
axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);

R.plot_real(all_markers, color_matrix, true);
plot(R.voronoi);
idx = 1;
FIRST = false;
while true
	x = [];
	y = [];
	X = [];
	while true
		if ~FIRST
			[xi, yi ,button] = ginput(1);
		end
		if size(X,1) > 2
			if ~isequal(button,1) % if enter is pressed
				break;
			end
		end
		FIRST = false;
		x = [x xi];
		y = [y yi];
		X = [x' y'];
		plot(x, y, '--ko');
	end
	large_obstacles{idx} = LARGE_OBSTACLE(X);
	clf;
	sgtitle("Select points to create obstacles")
	hold on;
	grid on;
	axis equal;
	axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);
	for i = 1:length(large_obstacles)
		large_obstacles{i}.plot();
	end
	R.plot_real(all_markers, color_matrix, true);
	plot(R.x_est(1), R.x_est(2), 'kx');
	plot(R.voronoi);
	idx = idx + 1;
	
	[xi,yi, button] = ginput(1);
	if ~isequal(button,1) % if enter is pressed
		break;
	else
		FIRST = true;
	end
end


[~, eigenvalues] = eig(R.P*3);
max_semiaxis = sqrt(max(diag(eigenvalues)));

% reduct the cell according to large obstacles
for i = 1:length(large_obstacles)
	voronoi_LO(large_obstacles{i}, R, max_semiaxis, parameters_simulation);
end

figure(2);
hold on;
grid on;
axis equal;
axis(parameters_simulation.size_map * 0.2 * [-1 1 -1 1]);
for i = 1:length(large_obstacles)
	large_obstacles{i}.plot();
end
<<<<<<< HEAD

for i = 1:size(couples_to_delete,1)
	plot(couples_to_delete(i,1), couples_to_delete(i,2), 'or');
end


for i = new_poly_voronoi.NumRegions
	% Kee the region in which the robot is
	tmp = rmboundary(new_poly_voronoi,1);
	if inpolygon(0,0,tmp.Vertices(:,1),tmp.Vertices(:,2))
		new_poly_voronoi = tmp;
		break;
	end
end



%{

 
   __  __       _           ____                          
  |  \/  | __ _| | _____   / ___|___  _ ____   _______  __
  | |\/| |/ _` | |/ / _ \ | |   / _ \| '_ \ \ / / _ \ \/ /
  | |  | | (_| |   <  __/ | |__| (_) | | | \ V /  __/>  < 
  |_|  |_|\__,_|_|\_\___|  \____\___/|_| |_|\_/ \___/_/\_\
                                                          
 

%}

new_points = setdiff(new_poly_voronoi.Vertices,poly_voronoi.Vertices,'rows','stable');
index = 1;
index_visible = 0;
visible_points = [];

region_to_delete = [];
for i = 1:size(new_points,1)-1
	% delete the area two by two points
	% define an area to delete for the first point
	dir = new_points(i,:) - [0,0];
	dir = dir/norm(dir);
	% create a point 1000 meters behind the intersection
	points_to_delete = [new_points(i,:) + 1000*dir; new_points(i,:)]; % matrix n by 2

	% define an area to delete for the second point
	dir = new_points(i+1,1:2) - [0,0];
	dir = dir/norm(dir);
	% create a point 1000 meters behind the intersection
	points_to_delete = [points_to_delete; [new_points(i+1,1:2) + 1000*dir ; new_points(i+1,1:2)]];

	% reorder the points to delete and create a polyshape
	points_to_delete = points_to_delete(convhull(points_to_delete(:,1:2)),1:2);
	region_to_delete{i} = polyshape(points_to_delete(:,1),points_to_delete(:,2));
end

for i = 1:length(region_to_delete)
	plot(region_to_delete{i});
end
total_region = region_to_delete{1};
% delete the area behind the intersection
for i = 2:length(region_to_delete)
	total_region = union(region_to_delete{i},total_region);
end

final_voronoi = subtract(new_poly_voronoi, total_region);
new_points = setdiff(final_voronoi.Vertices,new_poly_voronoi.Vertices,'rows','stable');

figure(3);
hold on;
grid on;
axis equal;
axis(10 * [-1 1 -1 1]);
plot(final_voronoi)
plot(0,0,'*k');
LO.plot();
for i = 1:size(new_points,1)
	plot(new_points(i,1),new_points(i,2),'og');
	plot([0,new_points(i,1)],[0,new_points(i,2)],'-g');
end
=======
R.plot_real(all_markers, color_matrix, true);
plot(R.voronoi);
>>>>>>> c04de915b6b0c7981ce206f81b0678c9e67f4e87
