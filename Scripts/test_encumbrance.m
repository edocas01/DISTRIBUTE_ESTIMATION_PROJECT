robot_i = [0;0];
delta_i = 1;
radiusc_i = 5*delta_i;
radiusrs_i = radiusc_i/2 - delta_i;\
robot_j = [5;0];
delta_j = 2.0;
radiusc_j = 5*delta_j;
radiusrs_j = radiusc_j/2 - delta_j;

delta = delta_i + delta_j;

[xcircle_i, ycircle_i] = Circle(robot_i(1), robot_i(2), delta_i);
[xcircle_j, ycircle_j] = Circle(robot_j(1), robot_j(2), delta_j);
[xcirclec_i, ycirclec_i] = Circle(robot_i(1), robot_i(2), radiusc_i);
[xcirclec_j, ycirclec_j] = Circle(robot_j(1), robot_j(2), radiusc_j);
[xcirclers_i, ycirclers_i] = Circle(robot_i(1), robot_i(2), radiusrs_i);
[xcirclers_j, ycirclers_j] = Circle(robot_j(1), robot_j(2), radiusrs_j);

close all;
figure()
subplot(2,1,1);
grid on;
axis equal;
hold on;
plot(xcircle_i, ycircle_i, 'r');
plot(xcircle_j, ycircle_j, 'b');
plot(robot_i(1), robot_i(2), 'r*');
plot(robot_j(1), robot_j(2), 'b*');
plot(xcirclec_i, ycirclec_i, 'r--');
plot(xcirclec_j, ycirclec_j, 'b--');
plot(xcirclers_i, ycirclers_i, 'r:');
plot(xcirclers_j, ycirclers_j, 'b:');


clc;
fprintf("Distance of the centers: %f\n", norm(robot_i - robot_j));
fprintf("Sum of the radius: %f\n", delta);
if norm(robot_i - robot_j)/2 < delta
	robot_j_tilde = robot_j + 4*(delta - norm(robot_i - robot_j)/2) * (robot_i - robot_j)/norm(robot_j - robot_i);
	fprintf("New distance of the centers: %f\n", norm(robot_i - robot_j_tilde));
	fprintf("Amount of displacement: %f\n", norm(robot_j - robot_j_tilde));
end
plot(robot_j_tilde(1), robot_j_tilde(2), 'g*');
[xcircle_j_tilde, ycircle_j_tilde] = Circle(robot_j_tilde(1), robot_j_tilde(2), delta_j);
plot(xcircle_j_tilde, ycircle_j_tilde, 'g');
[xcirclec_j_tilde, ycirclec_j_tilde] = Circle(robot_j_tilde(1), robot_j_tilde(2), radiusc_j);
plot(xcirclec_j_tilde, ycirclec_j_tilde, 'g--');
[xcirclers_j_tilde, ycirclers_j_tilde] = Circle(robot_j_tilde(1), robot_j_tilde(2), radiusrs_j);
plot(xcirclers_j_tilde, ycirclers_j_tilde, 'g:');

if norm(robot_i - robot_j)/2 < delta
	robot_i_tilde = robot_i + 2*(delta - norm(robot_i - robot_j)/2) * (robot_j - robot_i)/norm(robot_j - robot_i);
end
subplot(2,1,2);
hold on;
plot(xcircle_i, ycircle_i, 'r');
plot(xcircle_j, ycircle_j, 'b');
plot(robot_i(1), robot_i(2), 'r*');
plot(robot_j(1), robot_j(2), 'b*');
plot(xcirclec_i, ycirclec_i, 'r--');
plot(xcirclec_j, ycirclec_j, 'b--');
plot(xcirclers_i, ycirclers_i, 'r:');
plot(xcirclers_j, ycirclers_j, 'b:');
plot(robot_i_tilde(1), robot_i_tilde(2), 'g*');
[xcircle_i_tilde, ycircle_i_tilde] = Circle(robot_i_tilde(1), robot_i_tilde(2), delta_i);
plot(xcircle_i_tilde, ycircle_i_tilde, 'g');
[xcirclec_i_tilde, ycirclec_i_tilde] = Circle(robot_i_tilde(1), robot_i_tilde(2), radiusc_i);
plot(xcirclec_i_tilde, ycirclec_i_tilde, 'g--');
[xcirclers_i_tilde, ycirclers_i_tilde] = Circle(robot_i_tilde(1), robot_i_tilde(2), radiusrs_i);
plot(xcirclers_i_tilde, ycirclers_i_tilde, 'g:');
grid on;
axis equal;
