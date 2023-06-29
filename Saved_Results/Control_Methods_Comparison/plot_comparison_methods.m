clear;
close all;
clc;

flag_drag = false;

%% Load variables: DeePC controller

if flag_drag
    load("DeePC_step_drag.mat")
else
    load("DeePC_step.mat")
end

% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 3;
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

time_comp_deepc = solve_time_out;

%% Load variables: LQR controller
if flag_drag
    load("LQR_step_drag.mat")
else
    load("LQR_step.mat")
end

% Specific variables
u_lqr = [total_thrust_cmd body_rates_cmd].';

x_state_lqr = full_state_meas(:,1:9).';

x_lqr = x_state_lqr(1,:);
y_lqr = x_state_lqr(2,:);
z_lqr = x_state_lqr(3,:);

roll_lqr = x_state_lqr(7,:)*180/pi;
pitch_lqr = x_state_lqr(8,:)*180/pi;
yaw_lqr = x_state_lqr(9,:)*180/pi;

time_comp_lqr = solve_time_out;

% Remove unnecessary variables
clear total_thrust_cmd body_rates_cmd full_state_meas full_state_act...
    run_time param r_out Tini solve_time_out start_time

%% Plots
% % Position XY
% myFigure1 = figure();
% plot(time, x_deepc, 'LineWidth',1.5)
% hold on
% plot(time, x_lqr, 'LineWidth',1.5)
% hold on
% plot(time, y_deepc, 'LineWidth',1.5)
% hold on
% plot(time, y_lqr, 'LineWidth',1.5)
% hold on
% plot(time, r(1,:), '--', 'LineWidth',1.5)
% ylabel('Position [m]') 
% xlabel('Time [s]')
% legend('X_{DeePC}','X_{LQR}','Y_{DeePC}','Y_{LQR}','Ref','Location','northeast')
% grid on
% xlim([0 20])
% 
% % Position Z
% myFigure2 = figure();
% plot(time, z_deepc, 'LineWidth',1.5)
% hold on
% plot(time, z_lqr, 'LineWidth',1.5)
% hold on
% plot(time, r(3,:), '--', 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('DeePC','LQR','ref','Location','best')
% grid on
% xlim([0 20])

% % Euler Angles
% myFigure3 = figure();
% plot(time, roll_deepc, 'LineWidth',1.5)
% hold on
% plot(time, roll_lqr, 'LineWidth',1.5)
% hold on
% plot(time, pitch_deepc, 'LineWidth',1.5)
% hold on
% plot(time, pitch_lqr, 'LineWidth',1.5)
% hold on
% plot(time, yaw_deepc, 'LineWidth',1.5)
% hold on
% plot(time, yaw_lqr, 'LineWidth',1.5)
% ylabel('Euler Angles [\circ]') 
% xlabel('Time [s]')
% legend('\phi_{DeePC}','\phi_{LQR}','\theta_{DeePC}','\theta_{LQR}','\psi_{DeePC}','\psi_{LQR}','Location','southeast')
% grid on
% xlim([0 20])

% saveFigAsPDF(myFigure1,'Figures_PDF/XY_stepXYZ_model2');
% saveFigAsPDF(myFigure2,'Figures_PDF/Z_stepXYZ_model2');
% saveFigAsPDF(myFigure3,'Figures_PDF/Euler_stepXYZ_model2');


% Position XY
myFigure1 = figure();
subplot(3,1,1)
plot(time, x_deepc, 'LineWidth',1.5)
hold on
plot(time, x_lqr, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position X [m]') 
legend('DeePC','LQR','Ref','Location','northeast')
grid on
xlim([0 20])

subplot(3,1,2)
plot(time, y_deepc, 'LineWidth',1.5)
hold on
plot(time, y_lqr, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position Y [m]') 
legend('DeePC','LQR','Ref','Location','northeast')
grid on
xlim([0 20])

subplot(3,1,3)
plot(time, z_deepc, 'LineWidth',1.5)
hold on
plot(time, z_lqr, 'LineWidth',1.5)
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5)
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('DeePC','LQR','Ref','Location','best')
grid on
xlim([0 20])
ylim([-2.2 -1])


myFigure2 = figure();
subplot(3,1,1)
plot(time, roll_deepc, 'LineWidth',1.5)
hold on
plot(time, roll_lqr, 'LineWidth',1.5)
ylabel('\phi [\circ]') 
legend('DeePC','LQR','Location','best')
grid on
xlim([0 20])


subplot(3,1,2)
plot(time, pitch_deepc, 'LineWidth',1.5)
hold on
plot(time, pitch_lqr, 'LineWidth',1.5)
ylabel('\theta [\circ]') 
legend('DeePC','LQR','Location','best')
grid on
xlim([0 20])


subplot(3,1,3)
plot(time, yaw_deepc, 'LineWidth',1.5)
hold on
plot(time, yaw_lqr, 'LineWidth',1.5)
ylabel('\psi [\circ]') 
xlabel('Time [s]')
legend('DeePC','LQR','Location','southeast')
grid on
xlim([0 20])
ylim([-2 2])

% saveFigAsPDF(myFigure1,'Figures_PDF/LQR_vs_DeePC_pos');
% saveFigAsPDF(myFigure2,'Figures_PDF/LQR_vs_DeePC_ang');
% saveFigAsPDF(myFigure3,'Figures_PDF/Euler_stepXYZ_model2');

