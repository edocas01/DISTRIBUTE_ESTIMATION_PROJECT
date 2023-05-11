close all

P = [1,0.5;0.5,10];
c = [1,1];
point = [5,6];
[pt, rx, ry, th] = ellipse(c,P);
th = th*180/pi;
figure()
axis equal
hold on
plot(pt(1,:),pt(2,:),'b')
plot(c(1),c(2),'b*')
plot(point(1),point(2),'r*')

% plot line from point to center
plot([point(1),c(1)],[point(2),c(2)],'--k')

xlim("padded")
ylim("padded")

% find the closest point on the ellipse to the point
% TESTO TUTTI I PUNTI DELL' ELLISSE E TROVO IL PIU' VICINO -> NON E' IL MODO PIU' VELOCE
% MA E' IL PIU' SEMPLICE, prendo solo pochi punti per l'ellisse tipo 15 o 30 punti

[~,idx] = min(sum((pt-point').^2,1));
closest = pt(:,idx);
plot(closest(1),closest(2),'g*')

dist = norm(closest-point');
[x,y] = Circle(point(1),point(2),dist);
plot(x,y,'--k')

% plot the axes of the ellipse having length 2*rx and 2*ry
plot([c(1)-rx*cosd(th),c(1)+rx*cosd(th)],[c(2)-rx*sind(th),c(2)+rx*sind(th)],'--g')
plot([c(1)-ry*sind(th),c(1)+ry*sind(th)],[c(2)+ry*cosd(th),c(2)-ry*cosd(th)],'--g')

