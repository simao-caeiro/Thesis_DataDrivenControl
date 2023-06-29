clear;
close all;
clc;

flag_drag = false;

%% Load variables: DeePC controller

load("DeePC_ang_controller.mat")


% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 2;
sim_time = start_time + run_time;

time = 0:param.sample_time_measurements_full_state:sim_time;

% Specific variables
u_deepc = [total_thrust_cmd body_rates_cmd].';

x_state_deepc = full_state_meas(:,1:9).';

x_deepc = x_state_deepc(1,:);
y_deepc = x_state_deepc(2,:);
z_deepc = x_state_deepc(3,:);

roll_deepc = x_state_deepc(7,:)*180/pi;
pitch_deepc = x_state_deepc(8,:)*180/pi;
yaw_deepc = x_state_deepc(9,:)*180/pi;

% time_comp_deepc = solve_time_out;

%% Load variables: LQR controller

load("LQR_ang_controller.mat")


% Specific variables
u_lqr = [total_thrust_cmd body_rates_cmd].';

x_state_lqr = full_state_meas(:,1:9).';

x_lqr = x_state_lqr(1,:);
y_lqr = x_state_lqr(2,:);
z_lqr = x_state_lqr(3,:);

roll_lqr = x_state_lqr(7,:)*180/pi;
pitch_lqr = x_state_lqr(8,:)*180/pi;
yaw_lqr = x_state_lqr(9,:)*180/pi;

% time_comp_lqr = solve_time_out;

%% Load variables: MPC controller

load("MPC_ang_controller.mat")


% Specific variables
u_mpc = [total_thrust_cmd body_rates_cmd].';

x_state_mpc = full_state_meas(:,1:9).';
% x_state_mpc = full_state_meas(1:9,:);

x_mpc = x_state_mpc(1,:);
y_mpc = x_state_mpc(2,:);
z_mpc = x_state_mpc(3,:);

roll_mpc = x_state_mpc(7,:)*180/pi;
pitch_mpc = x_state_mpc(8,:)*180/pi;
yaw_mpc = x_state_mpc(9,:)*180/pi;

% time_comp_mpc = solve_time_out;

% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

%% Plots
% Position XY
myFigure1 = figure();
subplot(3,1,1)
plot(time, x_deepc, 'LineWidth',1.5)
hold on
plot(time, x_lqr, 'LineWidth',1.5)
hold on
plot(time, x_mpc, 'LineWidth',1.5,'Color','#77AC30')
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5, 'Color','#EDB120')
ylabel('Position X [m]') 
legend('DeePC','LQR','MPC','Ref','Location','northwest')
grid on
xlim([0 8])
ylim([1 2.5])

subplot(3,1,2)
plot(time, y_deepc, 'LineWidth',1.5)
hold on
plot(time, y_lqr, 'LineWidth',1.5)
hold on
plot(time, y_mpc, 'LineWidth',1.5, 'Color','#77AC30')
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5, 'Color','#EDB120')
ylabel('Position Y [m]') 
legend('DeePC','LQR','MPC','Ref','Location','northwest')
grid on
xlim([0 8])
ylim([1 2.5])

subplot(3,1,3)
plot(time, z_deepc, 'LineWidth',1.5)
hold on
plot(time, z_lqr, 'LineWidth',1.5)
hold on
plot(time, z_mpc, 'LineWidth',1.5, 'Color','#77AC30')
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5, 'Color','#EDB120')
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('DeePC','LQR','MPC','Ref','Location','northwest')
grid on
xlim([0 8])
ylim([-2.2 -1])


myFigure2 = figure();
subplot(3,1,1)
plot(time, roll_deepc, 'LineWidth',1.5)
hold on
plot(time, roll_lqr, 'LineWidth',1.5)
hold on
plot(time, roll_mpc, 'LineWidth',1.5, 'Color','#77AC30')
ylabel('\phi [\circ]') 
legend('DeePC','LQR','MPC','Location','southwest')
grid on
xlim([0 8])


subplot(3,1,2)
plot(time, pitch_deepc, 'LineWidth',1.5)
hold on
plot(time, pitch_lqr, 'LineWidth',1.5)
hold on
plot(time, pitch_mpc, 'LineWidth',1.5, 'Color','#77AC30')
ylabel('\theta [\circ]') 
legend('DeePC','LQR','MPC','Location','southwest')
grid on
xlim([0 8])


subplot(3,1,3)
plot(time, yaw_deepc, 'LineWidth',1.5)
hold on
plot(time, yaw_lqr, 'LineWidth',1.5)
hold on
plot(time, yaw_mpc, 'LineWidth',1.5, 'Color','#77AC30')
ylabel('\psi [\circ]') 
xlabel('Time [s]')
legend('DeePC','LQR','MPC','Location','southwest')
grid on
xlim([0 8])
ylim([-4 2])

% saveFigAsPDF(myFigure1,'Figures_PDF/Ang_Controller_LQR_vs_DeePC_vs_MPC_pos');
% saveFigAsPDF(myFigure2,'Figures_PDF/Ang_Controller_LQR_vs_DeePC_vs_MPC_ang');