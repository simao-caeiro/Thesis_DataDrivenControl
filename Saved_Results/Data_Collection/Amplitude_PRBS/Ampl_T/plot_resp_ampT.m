clear;
close all;
clc;

%% Load variables: Standard response
load("3.mat") % 10^{-1}

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

%% Load variables: Response 1
load("1.mat") % 10^{-3}

% Specific variables
u_1 = [total_thrust_cmd body_rates_cmd].';

x_state_1 = full_state_act(:,1:9).';

x_1 = x_state_1(1,:);
y_1= x_state_1(2,:);
z_1 = x_state_1(3,:);

roll_1 = x_state_1(7,:)*180/pi;
pitch_1 = x_state_1(8,:)*180/pi;
yaw_1 = x_state_1(9,:)*180/pi;

%% Load variables: Response 2
load("2.mat") % 10^{-2}

% Specific variables
u_2 = [total_thrust_cmd body_rates_cmd].';

x_state_2 = full_state_act(:,1:9).';

x_2 = x_state_2(1,:);
y_2= x_state_2(2,:);
z_2 = x_state_2(3,:);

roll_2 = x_state_2(7,:)*180/pi;
pitch_2 = x_state_2(8,:)*180/pi;
yaw_2 = x_state_2(9,:)*180/pi;

%% Load variables: Response 4
load("4.mat") % 10^{0}

% Specific variables
u_4 = [total_thrust_cmd body_rates_cmd].';

x_state_4 = full_state_act(:,1:9).';

x_4 = x_state_4(1,:);
y_4= x_state_4(2,:);
z_4 = x_state_4(3,:);

roll_4 = x_state_4(7,:)*180/pi;
pitch_4 = x_state_4(8,:)*180/pi;
yaw_4 = x_state_4(9,:)*180/pi;

%% Load variables: Response 5
load("5.mat") % 10

% Specific variables
u_5 = [total_thrust_cmd body_rates_cmd].';

x_state_5 = full_state_act(:,1:9).';

x_5 = x_state_5(1,:);
y_5= x_state_5(2,:);
z_5 = x_state_5(3,:);

roll_5 = x_state_5(7,:)*180/pi;
pitch_5 = x_state_5(8,:)*180/pi;
yaw_5 = x_state_5(9,:)*180/pi;


%% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

% Position X
myFigure1 = figure();
plot(time, x_1, 'LineWidth',1.5)
hold on
plot(time, x_2, 'LineWidth',1.5)
hold on
plot(time, x_std, 'LineWidth',1.5)
hold on
plot(time, x_4, 'LineWidth',1.5)
hold on
plot(time, x_5, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position X [m]') 
xlabel('Time [s]')
legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','southeast')
grid on
xlim([0 10])
ylim([0 3])


% Position Y
myFigure2 = figure();
plot(time, y_1, 'LineWidth',1.5)
hold on
plot(time, y_2, 'LineWidth',1.5)
hold on
plot(time, y_std, 'LineWidth',1.5)
hold on
plot(time, y_4, 'LineWidth',1.5)
hold on
plot(time, y_5, 'LineWidth',1.5)
hold on
plot(time, r(2,:), '--', 'LineWidth',1.5)
ylabel('Position Y [m]') 
xlabel('Time [s]')
legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','best')
grid on
xlim([0 10])
ylim([0 3])


% Position Z
myFigure3 = figure();
plot(time, z_1, 'LineWidth',1.5)
hold on
plot(time, z_2, 'LineWidth',1.5)
hold on
plot(time, z_std, 'LineWidth',1.5)
hold on
plot(time, z_4, 'LineWidth',1.5)
hold on
plot(time, z_5, 'LineWidth',1.5)
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5)
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','northeast')
grid on
xlim([0 10])
ylim([-3 0])

% saveFigAsPDF(myFigure1,'Figures_PDF/resp_X_ampT');
% saveFigAsPDF(myFigure2,'Figures_PDF/resp_Y_ampT');
% saveFigAsPDF(myFigure3,'Figures_PDF/resp_Z_ampT');


% % Position X
% myFigure1 = figure();
% subplot(3,1,1)
% plot(time, x_1, 'LineWidth',1.5)
% hold on
% plot(time, x_2, 'LineWidth',1.5)
% hold on
% plot(time, x_std, 'LineWidth',1.5)
% hold on
% plot(time, x_4, 'LineWidth',1.5)
% hold on
% plot(time, x_5, 'LineWidth',1.5)
% hold on
% plot(time, r(1,:), '--', 'LineWidth',1.5)
% ylabel('Position X [m]') 
% legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','southeast')
% grid on
% xlim([0 10])
% ylim([0 3])
% 
% subplot(3,1,2)
% plot(time, y_1, 'LineWidth',1.5)
% hold on
% plot(time, y_2, 'LineWidth',1.5)
% hold on
% plot(time, y_std, 'LineWidth',1.5)
% hold on
% plot(time, y_4, 'LineWidth',1.5)
% hold on
% plot(time, y_5, 'LineWidth',1.5)
% hold on
% plot(time, r(2,:), '--', 'LineWidth',1.5)
% ylabel('Position Y [m]') 
% legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','best')
% grid on
% xlim([0 10])
% ylim([0 5])
% 
% subplot(3,1,3)
% plot(time, z_1, 'LineWidth',1.5)
% hold on
% plot(time, z_2, 'LineWidth',1.5)
% hold on
% plot(time, z_std, 'LineWidth',1.5)
% hold on
% plot(time, z_4, 'LineWidth',1.5)
% hold on
% plot(time, z_5, 'LineWidth',1.5)
% hold on
% plot(time, r(3,:), '--', 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('g*10^{-2}','5g*10^{-2}','g*10^{-1}','5g*10^{-1}','g','Ref','Location','northeast')
% grid on
% xlim([0 10])
% ylim([-3 0])
