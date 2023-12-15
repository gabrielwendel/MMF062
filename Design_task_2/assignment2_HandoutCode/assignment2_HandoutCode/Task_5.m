%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vehicle Dynamics, MMF062, 2023
% Vertical assignment, Task 3
% Gabriel Wendel, Erik Lydig
clear all;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plots driver 1 20/80 distribution
% Plot lateral acceleration as a function of steering angle
% load('skidpad driver 1 2080 use.mat')
load('skidpad driver 2 2080.mat')

% Edit loop interval, until a satisfiable plot is achieved, do not use all
% data points.
for i = 1:length(delta)
    % Some data filtering
    if delta(i) > 0 && accelerationLocal(i, 2) > 0 && position(i, 2) ~= 0
        lat_acc(i) = accelerationLocal(i, 2);
        steer_ang(i) = delta(i);
        x_pos(i) = position(i, 1);
        y_pos(i) = position(i, 2);
    else
        lat_acc(i) = 0;  % Set to 0 if conditions are not met
        steer_ang(i) = 0;
        x_pos(i) = 0;
        y_pos(i) = 0;
    end
end

% Trim arrays to remove zero values
lat_acc = lat_acc(lat_acc ~= 0);
steer_ang = steer_ang(steer_ang ~= 0);
x_pos = x_pos(x_pos ~= 0);
y_pos = y_pos(y_pos ~= 0);

figure(1)
plot(steer_ang,lat_acc)
title('Lateral acceleration as a function of steering angle 80/20 distribution','Interpreter','latex')
xlabel('Steering angle $\delta$ [$^{\circ}$]','Interpreter','latex')
ylabel('Lateral acceleration [$m/s^2$]','Interpreter','latex')

% Plot trajectories of the vehicle position (xy plot)
figure(2)
plot(x_pos,y_pos)
title('Car trajectory (xy position) 80/20 distribution','interpreter','latex')
xlabel('x position [m]','interpreter','latex')
ylabel('y position [m]','interpreter','latex')




%% Plots driver 2 80/20 distribution
% Plot lateral acceleration as a function of steering angle
load('skidpad driver 2 8020 use.mat')

% Edit loop interval, until a satisfiable plot is achieved, do not use all
% data points. 
for i = 1:50000
    if delta(i) > 0 && accelerationLocal(i, 2) > 0 && position(i, 2) ~= 0
        lat_acc(i) = accelerationLocal(i, 2);
        steer_ang(i) = delta(i);
        x_pos(i) = position(i, 1);
        y_pos(i) = position(i, 2);
    else
        lat_acc(i) = 0;  % Set to 0 if conditions are not met
        steer_ang(i) = 0;
        x_pos(i) = 0;
        y_pos(i) = 0;
    end
end

% Trim arrays to remove zero values
lat_acc = lat_acc(lat_acc ~= 0);
steer_ang = steer_ang(steer_ang ~= 0);
x_pos = x_pos(x_pos ~= 0);
y_pos = y_pos(y_pos ~= 0);

figure(3)
plot(steer_ang,lat_acc)
title('Lateral acceleration as a function of steering angle 80/20 distribution','Interpreter','latex')
xlabel('Steering angle $\delta$ [$^{\circ}$]','Interpreter','latex')
ylabel('Lateral acceleration [$m/s^2$]','Interpreter','latex')

% Plot trajectories of the vehicle position (xy plot)
figure(4)
plot(x_pos,y_pos)
title('Car trajectory (xy position) 80/20 distribution','interpreter','latex')
xlabel('x position [m]','interpreter','latex')
ylabel('y position [m]','interpreter','latex')


