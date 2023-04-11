clc;
clear;
close all;

addpath('Scripts');
addpath('Functions');
addpath('Classes');

config;

%% Test robot
rob = ROBOT(0,0,1,1);
tar = TARGET(0.7,0.5);


figure(1)
axis equal
rob.plot()
tar.plot()
legend




