clear;
close all;
clc;

%% Load variables: Standard response
load("Nonlinear.mat") % band = 1

% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 3;
sim_time = start_time + run_time;

time = 0:param.sample_time_measurements_full_state:sim_time;

% Specific variables
u_std = [total_thrust_cmd body_rates_cmd].';

x_state_std = full_state_act(:,1:9).';

x_std = x_state_std(1,:);
y_std = x_state_std(2,:);
z_std = x_state_std(3,:);

roll_std = x_state_std(7,:)*180/pi;
pitch_std = x_state_std(8,:)*180/pi;
yaw_std = x_state_std(9,:)*180/pi;

thrust_exc_std = exc_param.thrust_exc_signal;
rolld_exc_std = exc_param.rolld_exc_signal;
pitchd_exc_std = exc_param.pitchd_exc_signal;

T_data_std = u_data(1,:);
rolld_data_std = u_data(2,:)*180/pi;
pitchd_data_std = u_data(3,:)*180/pi;

x_data_std = y_data(1,:);
y_data_std = y_data(2,:);
z_data_std = y_data(3,:);

time_data = 0:param.sample_time_measurements_full_state:exc_param.exc_signal_time-0.04;

%% Load variables: Response 1
load("Nonlinear_band2.mat") % band = 1/2

% Specific variables
u_1 = [total_thrust_cmd body_rates_cmd].';

x_state_1 = full_state_act(:,1:9).';

x_1 = x_state_1(1,:);
y_1 = x_state_1(2,:);
z_1 = x_state_1(3,:);

roll_1 = x_state_1(7,:)*180/pi;
pitch_1 = x_state_1(8,:)*180/pi;
yaw_1 = x_state_1(9,:)*180/pi;

thrust_exc_1 = exc_param.thrust_exc_signal;
rolld_exc_1 = exc_param.rolld_exc_signal;
pitchd_exc_1 = exc_param.pitchd_exc_signal;

T_data_1 = u_data(1,:);
rolld_data_1 = u_data(2,:)*180/pi;
pitchd_data_1 = u_data(3,:)*180/pi;

x_data_1 = y_data(1,:);
y_data_1 = y_data(2,:);
z_data_1 = y_data(3,:);

%% Load variables: Response 2
load("Nonlinear_band4.mat") % band = 1/4

% Specific variables
u_2 = [total_thrust_cmd body_rates_cmd].';

x_state_2 = full_state_act(:,1:9).';

x_2 = x_state_2(1,:);
y_2= x_state_2(2,:);
z_2 = x_state_2(3,:);

roll_2 = x_state_2(7,:)*180/pi;
pitch_2 = x_state_2(8,:)*180/pi;
yaw_2 = x_state_2(9,:)*180/pi;

thrust_exc_2 = exc_param.thrust_exc_signal;
rolld_exc_2 = exc_param.rolld_exc_signal;
pitchd_exc_2= exc_param.pitchd_exc_signal;

T_data_2 = u_data(1,:);
rolld_data_2 = u_data(2,:)*180/pi;
pitchd_data_2 = u_data(3,:)*180/pi;

x_data_2 = y_data(1,:);
y_data_2 = y_data(2,:);
z_data_2 = y_data(3,:);

%% Load variables: Response 3
load("Nonlinear_band8.mat") % band = 1/8

% Specific variables
u_3 = [total_thrust_cmd body_rates_cmd].';

x_state_3 = full_state_act(:,1:9).';

x_3 = x_state_3(1,:);
y_3 = x_state_3(2,:);
z_3 = x_state_3(3,:);

roll_3 = x_state_3(7,:)*180/pi;
pitch_3 = x_state_3(8,:)*180/pi;
yaw_3 = x_state_3(9,:)*180/pi;

thrust_exc_3 = exc_param.thrust_exc_signal;
rolld_exc_3 = exc_param.rolld_exc_signal;
pitchd_exc_3= exc_param.pitchd_exc_signal;

T_data_3 = u_data(1,:);
rolld_data_3 = u_data(2,:)*180/pi;
pitchd_data_3 = u_data(3,:)*180/pi;

x_data_3 = y_data(1,:);
y_data_3 = y_data(2,:);
z_data_3 = y_data(3,:);


%% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

%% Plots

% Position X
myFigure1 = figure();
plot(time, x_std, 'LineWidth',1.5)
hold on
plot(time, x_1, 'LineWidth',1.5)
hold on
plot(time, x_2, 'LineWidth',1.5)
hold on
plot(time, x_3, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position X [m]') 
xlabel('Time [s]')
legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Ref','Location','best')
grid on
xlim([0 8])


% Position Y
myFigure2 = figure();
plot(time, y_std, 'LineWidth',1.5)
hold on
plot(time, y_1, 'LineWidth',1.5)
hold on
plot(time, y_2, 'LineWidth',1.5)
hold on
plot(time, y_3, 'LineWidth',1.5)
hold on
plot(time, r(2,:), '--', 'LineWidth',1.5)
ylabel('Position Y [m]') 
xlabel('Time [s]')
legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Ref','Location','best')
grid on
xlim([0 8])


% Position Z
myFigure3 = figure();
plot(time, z_std, 'LineWidth',1.5)
hold on
plot(time, z_1, 'LineWidth',1.5)
hold on
plot(time, z_2, 'LineWidth',1.5)
hold on
plot(time, z_3, 'LineWidth',1.5)
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5)
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Ref','Location','best')
grid on
xlim([0 8])


% saveFigAsPDF(myFigure1,'Figures_PDF/resp_X_band');
% saveFigAsPDF(myFigure2,'Figures_PDF/resp_Y_band');
% saveFigAsPDF(myFigure3,'Figures_PDF/resp_Z_band');

% figure()
% plot(time_data, x_data_std, 'LineWidth',1.5)
% hold on
% plot(time_data, x_data_1, 'LineWidth',1.5)
% hold on
% plot(time_data, x_data_2, 'LineWidth',1.5)
% hold on
% plot(time_data, x_data_3, 'LineWidth',1.5)
% ylabel('Position X [m]') 
% xlabel('Time [s]')
% legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Location','best')
% grid on
% xlim([0 inf])

% figure()
% plot(time_data, y_data_std, 'LineWidth',1.5)
% hold on
% plot(time_data, y_data_1, 'LineWidth',1.5)
% hold on
% plot(time_data, y_data_2, 'LineWidth',1.5)
% hold on
% plot(time_data, y_data_3, 'LineWidth',1.5)
% ylabel('Position Y [m]') 
% xlabel('Time [s]')
% legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Location','best')
% grid on
% xlim([0 inf])
% 
% figure()
% plot(time_data, z_data_std, 'LineWidth',1.5)
% hold on
% plot(time_data, z_data_1, 'LineWidth',1.5)
% hold on
% plot(time_data, z_data_2, 'LineWidth',1.5)
% hold on
% plot(time_data, z_data_3, 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Location','best')
% grid on
% xlim([0 inf])
% 
% 
% figure()
% plot(time_data, T_data_std, 'LineWidth',1.5)
% hold on
% plot(time_data, T_data_1, 'LineWidth',1.5)
% hold on
% plot(time_data, T_data_2, 'LineWidth',1.5)
% hold on
% plot(time_data, T_data_3, 'LineWidth',1.5)
% ylabel('Thrust [N]') 
% xlabel('Time [s]')
% legend('B = 1','B = 1/2','B = 1/4','B = 1/8','Location','best')
% grid on
% xlim([0 inf])

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

