func = @(x,y,r,x_t,y_t) exp(-r/200*(-r + sqrt((x-x_t)^2 + (y-y_t)^2))^2);
% plot the 3d function
figure(1)
fsurf(@(x,y)func(x,y,5,0,0),[-10,10,-10,10])