%% Design Task 2
% Gabriel Wendel, Erik Lydig
close all;
clear all;
clc
%% Task 1.2, 1.3, 1.4
% Load Saab 9-3 parameters
run('InitModel.m')

% Given length ratios
length_ratio = [0.37 0.63 0.47];

l_f = [length_ratio(1)*vehicleData.L length_ratio(2)*vehicleData.L...
    length_ratio(3)*vehicleData.L];
l_r = [(1-length_ratio(1))*vehicleData.L (1-length_ratio(2))*vehicleData.L...
    (1-length_ratio(3))*vehicleData.L];

% Given parameters
c0 = 30.7;              % [1/rad]
c1 = -0.00235;          % [1/(N*rad)]
v_x = 100/3.6;          % velocity in x-direction [m/s]
acc_lat = 4;            % lateral acceleration [m/s^2]

% Initialize arrays
K_u = zeros(1,length(length_ratio));
v_xcrit = zeros(1,length(length_ratio));
v_xchar = zeros(1,length(length_ratio));
steer_ang = zeros(1,length(length_ratio));

% Loop over the load cases
for i=1:length(length_ratio)

    % Calculate longitudinal force on front- and rear wheel
    F_fz = vehicleData.m * vehicleData.g * (1-length_ratio(i))/2; % One wheel
    F_rz = vehicleData.m * vehicleData.g * length_ratio(i)/2;     % One wheel
    
    % Using longitudinal force, determine cornering stiffness
    C_f = (c0*F_fz + c1 * F_fz^2)*2; % Multiply by two to get for rear axle
    C_r = (c0*F_rz + c1 * F_rz^2)*2; % Multiply by two to get for rear axle
    
    % Understeer gradients
    K_u(i) = ((C_r*l_r(i))-(C_f*l_f(i))) / (C_f*C_r*vehicleData.L); % [1/N] or [rad/N] or [rad/(m/s^2)]

    % Critical and characteristic speeds using understeer gradients
    v_xcrit(i) = sqrt(vehicleData.L / (-K_u(i)*vehicleData.m));
    v_xchar(i) = sqrt(vehicleData.L / (K_u(i)*vehicleData.m));

    % Steering angle [rad]
    steer_ang(i) = acc_lat*(vehicleData.L + K_u(i)*vehicleData.m*v_x^2) / v_x^2; % From Equ 4.21 in Compendium
   
end

%% Task 1.5: Steering wheel angle plotted against varying operating points

radius = 200;                 % Curve radius [m]
v_x_points = 1:0.01:50;       % Array of operating points (velocity in x-direction) [m/s]

% Calculate steering angle using function "steering_angles"
steer_ang_1 = Steering_func(v_x_points, radius, K_u(1), vehicleData);
steer_ang_2 = Steering_func(v_x_points, radius, K_u(2), vehicleData);
steer_ang_3 = Steering_func(v_x_points, radius, K_u(3), vehicleData);

% Plot steering angle [rad] as a function of operating point [v_x] for all
% three length ratios
figure(1)
hold on
plot(v_x_points, steer_ang_1)
plot(v_x_points, steer_ang_2)
plot(v_x_points, steer_ang_3)
ylim([0, 0.45]);
xlabel('Operating points, $v_x$ [m/s]','Interpreter','latex')
ylabel('Steering Angle [rad]','Interpreter','latex')
legend('Load case 1','Load case 2', 'Load case 3','Interpreter','latex')
title('Steering Angle vs Operating points, $v_x$','Interpreter','latex')


% Function that computes Steering Angle for given operating point (v_x),
% curve radius, understeer gradient and vehicle data (Saab 9-3)
function steering_angles = Steering_func(v_x_points, radius, K_u, vehicleData)
    steering_angles = (vehicleData.L/radius + K_u*(vehicleData.m.*v_x_points.^2) / ...
        radius)*vehicleData.steeringRatio;
end




