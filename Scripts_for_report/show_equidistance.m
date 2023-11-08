clear all;
close all;
clc;
config;
settings_scripts;

% show the function to keep a given distance with respect to the robot
r = 3;
x_t = 0;
y_t = 0;
set(gcf, 'Position', get(0, 'Screensize'));
func = @(x,y,r,x_t,y_t) exp(-r/4*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2); 
fig = figure(1);
fsurf(@(x,y)func(x,y,r,x_t,y_t),[-6,6,-6,6],'EdgeColor','none')
camlight
title('Formation control function');
xlabel('x [m]');
ylabel('y [m]');
zlabel('$\phi(x,y)$')
export_fig(fig,'IMAGES/CONTROL_PLANNING/formation_function.png');
saveas(fig,'IMAGES/CONTROL_PLANNING/formation_function.fig');

% show simulation on equidistance
N = 8;
% generate the points on the circumference
points = [];
r = 3;
target = [0;0];

[xc,yc] = Circle(0, 0, r);

for i = 1:N
	tmp = rand()*2*pi;
	points(end+1,:) = [r*cos(tmp); r*sin(tmp)];
end

myVideo = VideoWriter('IMAGES/CONTROL_PLANNING/formation_control.avi');
myVideo.FrameRate = 6;  
open(myVideo);

figure(1);
set(gcf, 'Position', get(0, 'Screensize'));
hold on;
grid on;
axis equal;
xlim(["padded"])
ylim(["padded"])
plot(points(:,1), points(:,2), 'o', 'MarkerSize', 10, 'LineWidth', 2,'DisplayName', 'Robots');
plot(target(1), target(2), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');
plot(xc,yc,'-k','DisplayName', 'Formation cirlce');
xlabel('x [m]');
ylabel('y [m]');
title('Initial configuration');
legend
for i = 1:20
	points = decide_circle_barycenter(points, r, target);
	clf;
	hold on;
	grid on;
	axis equal;
	plot(points(:,1), points(:,2), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Robots');
	plot(target(1), target(2), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Target');
	plot(xc,yc,'-k', 'DisplayName', 'Formation cirlce');
	xlim(["padded"])
	ylim(["padded"])
	xlabel('x [m]');
	ylabel('y [m]');
	title(['Iterarion: ', num2str(i)]);
    legend
	pause(0.1);
	frame = getframe(gcf); %get frame
	writeVideo(myVideo, frame);
end
close(myVideo)



function center = decide_circle_barycenter(points, r, target)
	radius = r;
	center = [];

	for i = 1:length(points(:,1))
		new_points = points;
		tmp = new_points(i,:);
		new_points(i,:) = [];
		new_points = [new_points; tmp];

		my_idx = length(new_points(:,1));
		% compute the angles of the robots wrt target in order to find the "order"
		angles = wrapTo2Pi(atan2(new_points(:,2) - target(2), new_points(:,1) - target(1)));
		[~,idx] = sort(angles);
		
		% find the closest robots on the circle:
		% (first component is the anticlockwise neighbor starting from the horizontal axis)
		my_order = find(idx == my_idx);
		if my_order == 1
			angle_neighbors = [angles(idx(end),:); angles(idx(2),:)];
		elseif my_order == length(idx)
			angle_neighbors = [angles(idx(end-1),:); angles(idx(1),:)];
		else
			angle_neighbors = [angles(idx(my_order-1),:); angles(idx(my_order+1),:)];
		end
		
		% compute the angular distance between the two neighbors
		% notice that the angle has to be taken in the correct order:
		% from the previos to the next robot
		if (angles(my_idx) > angle_neighbors(1))
			angle_distance(1) = angles(my_idx) - angle_neighbors(1);
		else
			angle_distance(1) = angles(my_idx) - angle_neighbors(1) + 2*pi;
		end

		if (angle_neighbors(2) > angles(my_idx))
			angle_distance(2) = angle_neighbors(2) - angles(my_idx);
		else
			angle_distance(2) = angle_neighbors(2) - angles(my_idx) + 2*pi;
		end

		% apply the control law imposing a new angle accordingly to the difference
		% between the two angle_distances

		if angle_distance(1) <= angle_distance(2)
			% the robot has to move anticlockwise
			new_angle = angles(my_idx) + (angle_distance(2) - angle_distance(1))/3;
		else
			% the robot has to move clockwise
			new_angle = angles(my_idx) - (angle_distance(1) - angle_distance(2))/3;
		end

		% compute the center (it has to the circumference)
		center(end+1,:) = target + radius*[cos(new_angle); sin(new_angle)];
	end
end
