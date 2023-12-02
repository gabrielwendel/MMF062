%% Design Task 1 - Vehicle Dynamics
% Gabriel Wendel och Erik Lydig
clear all;
close all;
clc;

%% Task 1 a)
trac_force_step = 0.001;

road_condition = [1 2 3];

longitudinal_slip = 0:trac_force_step:1;

norm_trac_force = zeros(3,length(longitudinal_slip));
surface_type = cell(1,length(road_condition));

for i=1:3

    [norm_trac_force(i,:),surface_type{i}] = Sub_magic_tireformula(longitudinal_slip,road_condition(i));
    
end

figure(1)
plot(longitudinal_slip, norm_trac_force)
title('\textbf{Normalized longitudinal tyre force vs Slip, $s_x$}',...
    'Interpreter','latex')
xlabel('Longitudinal slip','interpreter','latex')
ylabel('Normalized traction force, $\mu$','interpreter','latex')
legend(surface_type{1},surface_type{2},surface_type{3},'interpreter','latex')

slip_data = 0:0.05:0.95;
norm_trac_force_data = [0.0 0.25 0.53 0.77 0.89 0.95 0.94 0.92 0.90 0.86...
                        0.85 0.83 0.81 0.80 0.79 0.78 0.77 0.76 0.75 0.74];

fit_trac_force = zeros(1,length(slip_data));

for i=1:length(slip_data)
    % Manually iterate value for C, D and E
    fit_trac_force(i) = magic_tyre_formula(slip_data(i),1.5,0.95,0);
end

figure(2)
plot(slip_data, norm_trac_force_data)
hold on
plot(slip_data,fit_trac_force)
hold off
title('\textbf{$\mu$ -slip experimental data vs Slip, $s_x$}','interpreter','latex')
xlabel('Longitudinal slip','interpreter','latex')
ylabel('Normalized traction force, $\mu$','interpreter','latex')
legend('Experimental data','Fitted data','interpreter','latex')


%% Funcions

% Magic Formula Tyre model
function mu = magic_tyre_formula(slip,C,D,E)
    K  = 3*pi/180;
    B  = atan(K)/(C*D);
    
    mu = D*sin(C*atan(B*slip*100-E*(B*slip*100-atan(B*slip*100))));
end

