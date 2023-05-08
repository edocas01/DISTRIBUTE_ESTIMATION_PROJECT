close all;
% Definisci la matrice di covarianza
cov_matrix1 = [1, 0.2; 0.2, 1];
cov_matrix2 = [0.8, 0; 0, 0.8];

% Genera campioni casuali dalla distribuzione multivariata
num_samples = 100;
mu1 = [2,4];
samples1 = mu1 + mvnrnd([0, 0], cov_matrix1, num_samples);
mu2 = [10,10];
samples2 = mu2 + mvnrnd([0, 0], cov_matrix2, num_samples);

% Estrai le coordinate x e y dei campioni
x1 = samples1(:, 1);
y1 = samples1(:, 2);
x2 = samples2(:, 1);
y2 = samples2(:, 2);


% Disegna il punto
figure(1);
hold on
plot(x1, y1, 'b.');  % 'b.' specifica il colore e il tipo di marker
hold on
plot(x2, y2, 'k.');  % 'b.' specifica il colore e il tipo di marker
% Aggiungi etichette agli assi
xlabel('X');
ylabel('Y');

% Aggiungi un titolo al grafico
title('Punto con matrice di covarianza');


pp1 = plotErrorEllipse(mu1, cov_matrix1, 0.99, 'r');
pp2 = plotErrorEllipse(mu2, cov_matrix2, 0.99, 'r');
plot(mu1(1),mu1(2),'r+', 'MarkerSize', 10, 'LineWidth', 2)
plot(mu2(1),mu2(2),'r+', 'MarkerSize', 10, 'LineWidth', 2)

X = [mu1(1);mu2(1)];
Y = [mu1(2);mu2(2)];
plot(X,Y,'-g')

dir = mu2 - mu1;
dir = dir/norm(dir);



function pp = plotErrorEllipse(mu, Sigma, p, color)

    s = -2 * log(1 - p);

    [V, D] = eig(Sigma * s);

    t = linspace(0, 2 * pi);
    a = (V * sqrt(D)) * [cos(t(:))'; sin(t(:))'];
 
    plot(a(1, :) + mu(1), a(2, :) + mu(2),color);
	pp = polyshape(a(1, :) + mu(1), a(2, :) + mu(2));
end