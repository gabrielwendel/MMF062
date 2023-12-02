function Sub_plot(t,a_vector,v_vector,s_vector,Tdrivf_vector,...
    Tdrivr_vector,Fzf_vector,Fzr_vector,slipf_vector,slipr_vector,...
    gearf_vector,gearr_vector,w_engf_vector,w_engr_vector)
% close all;
figure('Position', [415, 390, 560, 560])   %Distance, speed, acceleration, engine speed, gear
subplot(4,1,1)
plot(t,s_vector);
ylabel('Dist. [m]');
grid on
axFig1 = gca;
set(axFig1,'YColor','b');

subplot(4,1,2)
[fig2, ~, ~] = plotyy(t,v_vector*3.6, t, a_vector);
grid on;
legend('Speed','Acceleration','Location','SouthEast')
axes(fig2(1));
ylabel('Speed [km/h]')
axes(fig2(2));
ylabel('Acc. [m/s^2]');

subplot(4,1,3)
[fig3, ~, ~] = plotyy(t,w_engf_vector,t,gearf_vector);
grid on;
legend('Front Motor speed','Gear','Location','SouthEast')
axes(fig3(1));
ylabel('Motor speed [rad/s]');
axes(fig3(2));
ylabel('Gear [-]');
xlabel('Time [s]');

subplot(4,1,4)
[fig4, ~, ~] = plotyy(t,w_engr_vector,t,gearr_vector);
grid on;
legend('Rear Motor speed','Gear','Location','SouthEast')
axes(fig4(1));
ylabel('Motor speed [rad/s]');
axes(fig4(2));
ylabel('Gear [-]');
xlabel('Time [s]');

figure('Position', [991, 389, 560, 420])   %Force, torque, slip
subplot(3,1,1)
plot(t,Fzf_vector/1000,'r');
hold on
plot(t,Fzr_vector/1000,'g');
grid on
legend('front','rear');
ylabel('Fzf, Fzr [kN]');

subplot(3,1,2);
plot(t,Tdrivf_vector/1000,'r');
hold on;
plot(t,Tdrivr_vector/1000,'g');
grid on;
legend('front','rear');
ylabel('Tf, Tr [kNm]');

subplot(3,1,3)
plot(t,slipf_vector,'r');
hold on;
plot(t,slipr_vector,'g');
grid on;
legend('front','rear');
ylabel('Slip_f, Slip_r');
xlabel('Time [s]');
maxSlip = max([slipf_vector,slipr_vector]);
slipUpperBound = 1.05*maxSlip;
axis([min(t) max(t) 0 slipUpperBound])

end