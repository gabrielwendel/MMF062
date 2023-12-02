clear all
close all
clc

data1 = load("Data2_3.mat");
ay1 = data1.ay;
t = data1.t;
data2 = load("Data2_3_2.mat");
ay2 = data2.ay;
data3 = load("Data2_3_3.mat");
ay3 = data3.ay;


%% Vehicle lateral acceleration
% figure('Name','Lateral Acceleration');
hold on
plot(t,ay1,'DisplayName','Case 1','LineWidth',2)
plot(t,ay2,'DisplayName','Case 2','LineWidth',2)
plot(t,ay3,'DisplayName','Case 3','LineWidth',2)
grid on;
xlabel('Time [s]')
ylabel('Lateral acceleration [m/s^2]')
legend('Location', 'south east');
