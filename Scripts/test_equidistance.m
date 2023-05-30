
positions = [0,0.1, 0.11, 1.2, 150];
for i = 1:10
	for j = 2:length(positions) - 1
		distance(j,1) = abs(positions(j) - positions(j-1));
		distance(j,2) = abs(positions(j+1) - positions(j));
	end
	for j = 2:length(positions) - 1
		if distance(j,1) > distance(j,2)
			positions(j) = positions(j) - (distance(j,1) - distance(j,2))/2;
		else
			positions(j) = positions(j) + (distance(j,2) - distance(j,1))/2;
		end
	end
	figure(1) 
	clf;
	plot(positions(1), 0, 'xr');
	hold on;
	plot(positions(end), 0, 'xr');
	plot(positions(2),0,'xb');
	plot(positions(3),0,'xg');
	plot(positions(4),0,'xk');
	xlim("padded");
	ylim([-1,1]);
	pause(0.1);
end