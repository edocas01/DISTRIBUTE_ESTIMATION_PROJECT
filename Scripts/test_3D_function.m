% For circle formation
func = @(x,y,x_t,y_t) exp(-((x-x_t)^2 + (y-y_t)^2));
% plot the 3d function
figure(1); clf
fsurf(@(x,y)func(x,y,10,13),[-20,20,-20,20])
