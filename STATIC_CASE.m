%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
close all
clearvars

addpath("Classes\")
addpath("Functions\")

rng('default');

%% Entities definition
% Target
x_T = 0;
y_T = 0;
ComRadius_T = 1;

T = TARGET();
T = T.setPosition(x_T, y_T);
T = T.setComRadius(ComRadius_T);

% Robots
n_robot = 6;
xR_min = -5;
xR_max = 5;
yR_min = -5;
yR_max = 5;
ComRadius_R = 1 + rand(1, n_robot); % Comunication radii [1 - 2] 

for i = 1 : n_robot
	R.(strcat("R",num2str(i))) = ROBOT();
	R.(strcat("R",num2str(i))) = R.(strcat("R",num2str(i))).setPosition(xR_min + (xR_max - xR_min) * rand(), yR_min + (yR_max - yR_min) * rand());
	R.(strcat("R",num2str(i))) = R.(strcat("R",num2str(i))).setComRadius(ComRadius_R(i));
end

%% Plot the entities
figure(1)
hold on
grid on
axis equal
axis([-6 6 -6 6])
xlabel("x")
ylabel("y")
title("Entities")
plot(T.x, T.y, 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r')
Circle(T.x, T.y, T.ComRadius)
for i = 1 : n_robot
    plot(R.(strcat("R",num2str(i))).x, R.(strcat("R",num2str(i))).y, 'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b')
    	Circle(R.(strcat("R",num2str(i))).x, R.(strcat("R",num2str(i))).y, R.(strcat("R",num2str(i))).ComRadius)
end
hold off



