close all;
clc;
clearvars;

points = 10;
range = 5;
for i = 1:points
	x(i) = (rand() - 0.5)*range;
	y(i) = (rand() - 0.5)*range;
end

P = [x', y'];
% Find the voroni map
[vin,cin] = voronoin(P);
[vx,vy] = voronoi(x,y);

figure()
plot(vx,vy,'b')
hold on 
plot(x, y, 'or');
plot(vin(:,1), vin(:,2), 'ok')
% define of the vertex of voronoi in a matrix
v = zeros(length(vx(1,:))*2,2);
v = [vx(1,:)', vy(1,:)'; vx(2,:)', vy(2,:)'];
% remove the duplicate points
v = unique(v, 'rows');
plot(v(:,1), v(:,2), '.g');

% remove the first element of vin if it's infinite
vin_noinf = vin;
cin_noinf = cin;
if (isinf(vin(1,1)) || isinf(vin(1,2)))
	vin_noinf(1,:) = [];
	for i = 1:length(cin)
		cin_noinf{i}(find(cin{i} == 1)) = [];
	end
end

% find the points that are not conisdered in vin_noinf but are in v


% compute voronoi and voronoin
% find all the valid points in voronoin
% compute the intersections between the "infinite" points and the circle
% inteserection between the 2 polylines