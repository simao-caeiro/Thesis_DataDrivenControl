clear;
close all;
clc;

%% Load variables: Nonlinear controller
load("Nonlinear.mat")

% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 2;
sim_time = start_time + run_time;

time = 0:param.sample_time_measurements_full_state:sim_time;

% Specific variables
u_nl = [total_thrust_cmd body_rates_cmd].';

x_state_nl = full_state_meas(:,1:9).';

x_nl = x_state_nl(1,:);
y_nl = x_state_nl(2,:);
z_nl = x_state_nl(3,:);

roll_nl = x_state_nl(7,:)*180/pi;
pitch_nl = x_state_nl(8,:)*180/pi;
yaw_nl = x_state_nl(9,:)*180/pi;

thrust_exc_nl = exc_param.thrust_exc_signal;
rolld_exc_nl = exc_param.rolld_exc_signal;
pitchd_exc_nl = exc_param.pitchd_exc_signal;

thrust_collected = total_thrust_collected;
rolld_collected = eulerd_collected(:,1)*180/pi;
pitchd_collected = eulerd_collected(:,2)*180/pi;

thrust_PRBS = total_thrust_PRBS;
rolld_PRBS = eulerd_PRBS(:,1)*180/pi;
pitchd_PRBS = eulerd_PRBS(:,2)*180/pi;

thrust_LQR = total_thrust_LQR;
rolld_LQR = eulerd_LQR(:,1)*180/pi;
pitchd_LQR = eulerd_LQR(:,2)*180/pi;

T_data_nl = u_data(1,:);
rolld_data_nl = u_data(2,:)*180/pi;
pitchd_data_nl = u_data(3,:)*180/pi;

x_data_nl = y_data(1,:);
y_data_nl = y_data(2,:);
z_data_nl = y_data(3,:);

time_data = 0:param.sample_time_measurements_full_state:exc_param.exc_signal_time-0.04;


%% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

%% Plots

myFigure1 = figure();
plot(time_data, x_data_nl, 'LineWidth',1.5)
hold on
plot(time_data, y_data_nl, 'LineWidth',1.5)
hold on
plot(time_data, z_data_nl, 'LineWidth',1.5)
ylabel('Position [m]') 
xlabel('Time [s]')
legend('X','Y','Z','Location','best')
grid on
xlim([0 inf])

% figure()
% plot(time_data, T_data_nl, 'LineWidth',1.5)
% ylabel('Thrust [N]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, rolld_data_nl, 'LineWidth',1.5)
% ylabel('Roll_d [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, pitchd_data_nl, 'LineWidth',1.5)
% ylabel('Pitch_d [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])

% myFigure2 = figure();
% plot(time_data, T_data_nl, 'LineWidth',1.5)
% ylabel('Thrust [N]')
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% myFigure3 = figure();
% plot(time_data, rolld_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_data_nl, 'LineWidth',1.5)
% legend('\phi_d','\theta_d','Location','northwest')
% ylabel('Euler Angles [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])



% myFigure2 = figure();
% plot(time_data, thrust_collected, 'LineWidth',1.5)
% ylabel('Thrust [N]')
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% myFigure3 = figure();
% plot(time_data, rolld_collected, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_collected, 'LineWidth',1.5)
% legend('\phi_d','\theta_d','Location','northwest')
% ylabel('Euler Angles [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% %%%%%%%%%%%
% myFigure4 = figure();
% plot(time_data, thrust_PRBS, 'LineWidth',1.5)
% ylabel('Thrust [N]')
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% ylim([-1.5 1.5])
% 
% myFigure5 = figure();
% plot(time_data, rolld_PRBS, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_PRBS, 'LineWidth',1.5)
% legend('\phi_d','\theta_d','Location','northwest')
% ylabel('Euler Angles [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% ylim([-8 8])
% 
% %%%%%%%%%%
% myFigure6 = figure();
% plot(time_data, thrust_LQR, 'LineWidth',1.5)
% ylabel('Thrust [N]')
% xlabel('Time [s]')
% grid on
% xlim([0 inf])
% 
% myFigure7 = figure();
% plot(time_data, rolld_LQR, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_LQR, 'LineWidth',1.5)
% legend('\phi_d','\theta_d','Location','northwest')
% ylabel('Euler Angles [\circ]')  
% xlabel('Time [s]')
% grid on
% xlim([0 inf])


myFigure2 = figure();
subplot(3,1,1)
plot(time_data, thrust_LQR, 'LineWidth',1.5)
ylabel('Thrust [N]')
grid on
xlim([0 inf])

subplot(3,1,2)
plot(time_data, thrust_PRBS, 'LineWidth',1.5)
ylabel('Thrust [N]')
grid on
xlim([0 inf])
ylim([-1.5 1.5])

subplot(3,1,3)
plot(time_data, thrust_collected, 'LineWidth',1.5)
ylabel('Thrust [N]')
xlabel('Time [s]')
grid on
xlim([0 inf])


myFigure3 = figure();
subplot(3,1,1)
plot(time_data, rolld_LQR, 'LineWidth',1.5)
hold on
plot(time_data, pitchd_LQR, 'LineWidth',1.5)
legend('\phi_d','\theta_d','Location','northwest')
ylabel('Euler Angles [\circ]')  
grid on
xlim([0 inf])
ylim([-5 5])

subplot(3,1,2)
plot(time_data, rolld_PRBS, 'LineWidth',1.5)
hold on
plot(time_data, pitchd_PRBS, 'LineWidth',1.5)
legend('\phi_d','\theta_d','Location','northwest')
ylabel('Euler Angles [\circ]')  
grid on
xlim([0 inf])
ylim([-8 8])

subplot(3,1,3)
plot(time_data, rolld_collected, 'LineWidth',1.5)
hold on
plot(time_data, pitchd_collected, 'LineWidth',1.5)
legend('\phi_d','\theta_d','Location','northwest')
ylabel('Euler Angles [\circ]')  
xlabel('Time [s]')
grid on
xlim([0 inf])


saveFigAsPDF(myFigure1,'Figures_PDF/data_collected_pos');
saveFigAsPDF(myFigure2,'Figures_PDF/data_collected_T');
saveFigAsPDF(myFigure3,'Figures_PDF/data_collected_ang');



% saveFigAsPDF(myFigure1,'Figures_PDF/data_collected_pos');
% saveFigAsPDF(myFigure2,'Figures_PDF/data_collected_T');
% saveFigAsPDF(myFigure3,'Figures_PDF/data_collected_ang');
% saveFigAsPDF(myFigure4,'Figures_PDF/PRBS_T');
% saveFigAsPDF(myFigure5,'Figures_PDF/PRBS_ang');
% saveFigAsPDF(myFigure6,'Figures_PDF/LQR_T');
% saveFigAsPDF(myFigure7,'Figures_PDF/LQR_ang');
