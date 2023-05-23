% For circle formation
func = @(x,y,r,x_t,y_t) exp(-r/400*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2);
% plot the 3d function
figure(1); clf
fsurf(@(x,y)func(x,y,5,0,0),[-10,10,-10,10])

% % For square formation
% func = @(x,y,r,x_t,y_t) exp(-(1/100) * r * (-r + abs(x - x_t) + abs(y - y_t))^2);
% figure(2)
% fsurf(@(x,y)func(x,y,5,0,0), [-10,10,-10,10])