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

x_state_nl = full_state_act(:,1:9).';

x_nl = x_state_nl(1,:);
y_nl = x_state_nl(2,:);
z_nl = x_state_nl(3,:);

roll_nl = x_state_nl(7,:)*180/pi;
pitch_nl = x_state_nl(8,:)*180/pi;
yaw_nl = x_state_nl(9,:)*180/pi;

thrust_exc_nl = exc_param.thrust_exc_signal;
rolld_exc_nl = exc_param.rolld_exc_signal;
pitchd_exc_nl = exc_param.pitchd_exc_signal;

T_data_nl = u_data(1,:);
rolld_data_nl = u_data(2,:)*180/pi;
pitchd_data_nl = u_data(3,:)*180/pi;

x_data_nl = y_data(1,:);
y_data_nl = y_data(2,:);
z_data_nl = y_data(3,:);

time_data = 0:param.sample_time_measurements_full_state:exc_param.exc_signal_time-0.04;


%% Load variables: Linear controller
load("Nonlinear_no_circshift.mat")

% Specific variables
u_nl_nc = [total_thrust_cmd body_rates_cmd].';

x_state_nl_nc = full_state_act(:,1:9).';

x_nl_nc = x_state_nl_nc(1,:);
y_nl_nc= x_state_nl_nc(2,:);
z_nl_nc = x_state_nl_nc(3,:);

roll_nl_nc = x_state_nl_nc(7,:)*180/pi;
pitch_nl_nc = x_state_nl_nc(8,:)*180/pi;
yaw_nl_nc = x_state_nl_nc(9,:)*180/pi;

time_comp_nl_nc = solve_time_out;

thrust_exc_nl_nc = exc_param.thrust_exc_signal;
rolld_exc_nl_nc = exc_param.rolld_exc_signal;
pitchd_exc_nl_nc = exc_param.pitchd_exc_signal;

T_data_nl_nc = u_data(1,:);
rolld_data_nl_nc = u_data(2,:)*180/pi;
pitchd_data_nl_nc = u_data(3,:)*180/pi;

x_data_nl_nc = y_data(1,:);
y_data_nl_nc = y_data(2,:);
z_data_nl_nc = y_data(3,:);

% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

%% Plots

% % Positions
% f_pos = figure();
% f_pos.Position = [50 100 1450 450];
% t_pos = tiledlayout(1,3,'TileSpacing','tight','Padding','tight');
% title(t_pos,'Positions','FontWeight','bold')
% 
% nexttile
% plot(time, x_nl)
% hold on
% plot(time, x_l)
% hold on
% plot(time, r(1,:))
% ylabel('X [m]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','ref','Location','best')
% grid on
% 
% nexttile
% plot(time, y_nl)
% hold on
% plot(time, y_l)
% hold on
% plot(time, r(2,:))
% ylabel('Y [m]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','ref','Location','best')
% grid on
% 
% nexttile
% plot(time, z_nl)
% hold on
% plot(time, z_l)
% hold on
% plot(time, r(3,:))
% ylabel('Z [m]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','ref','Location','best')
% grid on


% % Euler Angles
% f_ang = figure();
% f_ang.Position = [50 100 1450 600];
% t_ang = tiledlayout(1,3,'TileSpacing','tight','Padding','tight');
% title(t_ang,'Euler Angles','FontWeight','bold')
% 
% nexttile
% plot(time, roll_nl)
% hold on
% plot(time, roll_l)
% ylabel('Phi [º]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','Location','best')
% grid on
% 
% nexttile
% plot(time, pitch_nl)
% hold on
% plot(time, pitch_l)
% ylabel('Theta [º]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','Location','best')
% grid on
% 
% nexttile
% plot(time, yaw_nl)
% hold on
% plot(time, yaw_l)
% ylabel('Psi [º]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','Location','best')
% grid on
% 
% 
% % Thrust
% figure()
% plot(time, u_nl(1,:))
% hold on
% plot(time, u_l(1,:))
% hold on
% plot(time, T_eq*ones(1,length(time)), '--')
% ylabel('T [N]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','ref','Location','best')
% grid on


% % Position XY
% myFigure1 = figure();
% plot(time, x_nl, 'LineWidth',1.5)
% hold on
% plot(time, x_nl_nc, 'LineWidth',1.5)
% hold on
% plot(time, y_nl, 'LineWidth',1.5)
% hold on
% plot(time, y_nl_nc, 'LineWidth',1.5)
% hold on
% plot(time, r(1,:), '--', 'LineWidth',1.5)
% ylabel('Position [m]') 
% xlabel('Time [s]')
% legend('X','X_{No Step4}','Y','Y_{No Step4}','Ref','Location','east')
% grid on
% xlim([0 8])
% ylim([0 2.5])
% 
% % Position Z
% myFigure2 = figure();
% plot(time, z_nl, 'LineWidth',1.5)
% hold on
% plot(time, z_nl_nc, 'LineWidth',1.5)
% hold on
% plot(time, r(3,:), '--', 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','ref','Location','best')
% grid on
% xlim([0 8])
% ylim([-2.5 0])

% % Euler Angles
% myFigure3 = figure();
% plot(time, roll_nl, 'LineWidth',1.5)
% hold on
% plot(time, roll_nl_nc, 'LineWidth',1.5)
% hold on
% plot(time, pitch_nl, 'LineWidth',1.5)
% hold on
% plot(time, pitch_nl_nc, 'LineWidth',1.5)
% hold on
% plot(time, yaw_nl, 'LineWidth',1.5)
% hold on
% plot(time, yaw_nl_nc, 'LineWidth',1.5)
% ylabel('Euler Angles [\circ]') 
% xlabel('Time [s]')
% legend('\phi_{nonlinear}','\phi_{linear}','\theta_{nonlinear}','\theta_{linear}','\psi_{nonlinear}','\psi_{linear}','Location','southeast')
% grid on
% xlim([0 8])


% Position XY
myFigure1 = figure();
subplot(3,1,1)
plot(time, x_nl, 'LineWidth',1.5)
hold on
plot(time, x_nl_nc, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position X [m]') 
ylim([0 2.5])
xlim([0 6])
legend('Standard','No Step4','ref','Location','best')
grid on

subplot(3,1,2)
plot(time, y_nl, 'LineWidth',1.5)
hold on
plot(time, y_nl_nc, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position Y [m]') 
ylim([0 2.5])
xlim([0 6])
legend('Standard','No Step4','ref','Location','best')
grid on

subplot(3,1,3)
plot(time, z_nl, 'LineWidth',1.5)
hold on
plot(time, z_nl_nc, 'LineWidth',1.5)
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5)
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('Standard','No Step4','ref','Location','best')
grid on
xlim([0 6])
ylim([-2.5 0])


saveFigAsPDF(myFigure1,'Figures_PDF/resp_XYZ_nocircshift');
% saveFigAsPDF(myFigure2,'Figures_PDF/resp_Z_nocircshift');



% figure()
% plot(time_data, x_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, x_data_nl_nc, 'LineWidth',1.5)
% ylabel('Position X [m]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, y_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, y_data_nl_nc, 'LineWidth',1.5)
% ylabel('Position Y [m]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, z_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, z_data_nl_nc, 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])


% figure()
% plot(time_data, T_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, T_data_nl_nc, 'LineWidth',1.5)
% ylabel('Thrust [N]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, rolld_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, rolld_data_nl_nc, 'LineWidth',1.5)
% ylabel('Roll_d [\circ]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% 
% figure()
% plot(time_data, pitchd_data_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_data_nl_nc, 'LineWidth',1.5)
% ylabel('Pitch_d [\circ]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% 
% figure()
% plot(time_data, thrust_exc_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, thrust_exc_nl_nc, 'LineWidth',1.5)
% ylabel('Thrust [N]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, rolld_exc_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, rolld_exc_nl_nc, 'LineWidth',1.5)
% ylabel('Roll_d [\circ]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])
% 
% 
% figure()
% plot(time_data, pitchd_exc_nl, 'LineWidth',1.5)
% hold on
% plot(time_data, pitchd_exc_nl_nc, 'LineWidth',1.5)
% ylabel('Pitch_d [\circ]') 
% xlabel('Time [s]')
% legend('Standard','No Step4','Location','best')
% grid on
% xlim([0 inf])


