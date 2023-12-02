%% Design Task 2
% Task 3.2
close all;
clear all;
clc
%%

% Load data from task 2.2 and pick peak lateral acceleration
data1_task_2 = load("Data2_3.mat");

ay1_task_2_peak = max(data1_task_2.ay);
t = data1_task_2.t;
data2_task_2 = load("Data2_3_2.mat");

ay2_task_2_peak = max(data2_task_2.ay);
data3_task_2 = load("Data2_3_3.mat");

ay3_task_2_peak = max(data3_task_2.ay);

task_2_peak = [ay1_task_2_peak ay2_task_2_peak ay3_task_2_peak];

% Task 3.2 data
data1_task_3 = load("Data3_2.mat");
ay1_task_3 = data1_task_3.ay;
ay1_task_3_peak = max(data1_task_3.ay);
data2_task_3 = load("Data3_2_2.mat");
ay2_task_3 = data2_task_3.ay;
ay2_task_3_peak = max(data2_task_3.ay);
data3_task_3 = load("Data3_2_3.mat");
ay3_task_3 = data3_task_3.ay;
ay3_task_3_peak = max(data3_task_3.ay);

task_3_peak = [ay1_task_3_peak ay2_task_3_peak ay3_task_3_peak];

peak_ratio = zeros(1,3);

% Compare peak lateral acceleration between Task 3.2 and Task 2.2. the
% difference is expressed in percentage
for i=1:3
    peak_ratio(i) = task_3_peak(i) / task_2_peak(i);
end

% Plot lateral acc
hold on
plot(t,ay1_task_3,'DisplayName','Case 1','LineWidth',2)
plot(t,ay2_task_3,'DisplayName','Case 2','LineWidth',2)
plot(t,ay3_task_3,'DisplayName','Case 3','LineWidth',2)
grid on;
xlabel('Time [s]')
ylabel('Lateral acceleration [m/s^2]')
title('Lateral acceleration, load transfer model')
legend('Location', 'south east');


