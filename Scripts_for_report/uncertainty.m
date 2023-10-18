close all;
num = 40;
% R = (rand(2,2) - 0.5)*num;
R = (eye(2,2))*num;
R = R*R';

for i = 1:10000
    P(:,i) = mvnrnd([0;0], R)';
end

figure(1)
hold on
plot(P(1,:),P(2,:),'or');
