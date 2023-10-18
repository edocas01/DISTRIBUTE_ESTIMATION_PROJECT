clc;
close all;
config;

index = 2;
distances = sum(abs(R{index}.voronoi.Vertices - R{index}.x_est').^2,2).^0.5;
[maxdist, idx] = max(distances);
vert = R{index}.voronoi.Vertices(idx,:);

figure(100);
hold on; axis equal; grid on;
R{index}.plot_real(all_markers, color_matrix, true);
plot(vert(1), vert(2), 'or');
plot(R{index}.voronoi);
