clear;
close all;
clc;

%% Load variables: Nonlinear+noise controller
load("Nonlinear_noise.mat")

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

time_comp_nl = solve_time_out;

%% Load variables: Nonlinear+noise controller
load("Nonlinear_drag_noise.mat")

% Specific variables
u_drag = [total_thrust_cmd body_rates_cmd].';

x_state_drag = full_state_meas(:,1:9).';

x_drag = x_state_drag(1,:);
y_drag= x_state_drag(2,:);
z_drag = x_state_drag(3,:);

roll_drag = x_state_drag(7,:)*180/pi;
pitch_drag = x_state_drag(8,:)*180/pi;
yaw_drag = x_state_drag(9,:)*180/pi;

time_comp_drag = solve_time_out;

%% Load variables: Nonlinear+noise controller
load("Nonlinear.mat")

% Specific variables
u_std = [total_thrust_cmd body_rates_cmd].';

x_state_std = full_state_meas(:,1:9).';

x_std = x_state_std(1,:);
y_std= x_state_std(2,:);
z_std = x_state_std(3,:);

roll_std = x_state_std(7,:)*180/pi;
pitch_std = x_state_std(8,:)*180/pi;
yaw_std = x_state_std(9,:)*180/pi;

time_comp_std = solve_time_out;

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


linewidth = 1.5;

% Position XYZ
myFigure4= figure();

subplot(3,1,1)
plot(time, x_nl, 'LineWidth',linewidth)
hold on
plot(time, x_drag, 'LineWidth',linewidth)
hold on
plot(time, r(1,:), '--', 'LineWidth',linewidth)
ylabel('Position X [m]') 
xlim([0 8])
ylim([0.8 2.2])
legend('Noise','Noise+Drag','Ref','Location','southeast')
grid on

subplot(3,1,2)
plot(time, y_nl, 'LineWidth',linewidth)
hold on
plot(time, y_drag, 'LineWidth',linewidth)
hold on
plot(time, r(2,:), '--', 'LineWidth',linewidth)
ylabel('Position Y [m]') 
xlim([0 8])
ylim([0.8 2.2])
legend('Noise','Noise+Drag','Ref','Location','southeast')
grid on

subplot(3,1,3)
plot(time, z_nl, 'LineWidth',linewidth)
hold on
plot(time, z_drag, 'LineWidth',linewidth)
hold on
plot(time, r(3,:), '--', 'LineWidth',linewidth)
ylabel('Position Z [m]') 
xlabel('Time [s]')
xlim([0 8])
ylim([-2.2 -0.8])
legend('Noise','Noise+Drag','Ref','Location','northeast')
grid on

% Euler Angles
myFigure5= figure();

subplot(3,1,1)
plot(time, roll_nl, 'LineWidth',linewidth)
hold on
plot(time, roll_drag, 'LineWidth',linewidth)
ylabel('\phi [\circ]') 
legend('Noise','Noise+Drag','Location','northeast')
grid on
xlim([0 8])

subplot(3,1,2)
plot(time, pitch_nl, 'LineWidth',linewidth)
hold on
plot(time, pitch_drag, 'LineWidth',linewidth)
ylabel('\theta [\circ]') 
legend('Noise','Noise+Drag','Location','southeast')
grid on
xlim([0 8])

subplot(3,1,3)
plot(time, yaw_nl, 'LineWidth',linewidth)
hold on
plot(time, yaw_drag, 'LineWidth',linewidth)
ylabel('\psi [\circ]') 
xlabel('Time [s]')
legend('Noise','Noise+Drag','Location','southeast')
grid on
xlim([0 8])
ylim([-2.5 2.5])

% saveFigAsPDF(myFigure4,'Figures_PDF/Position_drag_noise_stepXYZ_model2');
% saveFigAsPDF(myFigure5,'Figures_PDF/Orientation_drag_noise_stepXYZ_model2');

myFigure6= figure();

subplot(3,1,1)
plot(time, x_std, 'LineWidth',linewidth)
hold on
plot(time, x_nl, 'LineWidth',linewidth)
hold on
plot(time, r(1,:), '--', 'LineWidth',linewidth)
ylabel('Position X [m]') 
xlim([0 8])
ylim([0.8 2.2])
legend('Standard','Noise','Ref','Location','southeast')
grid on

subplot(3,1,2)
plot(time, y_std, 'LineWidth',linewidth)
hold on
plot(time, y_nl, 'LineWidth',linewidth)
hold on
plot(time, r(2,:), '--', 'LineWidth',linewidth)
ylabel('Position Y [m]') 
xlim([0 8])
ylim([0.8 2.2])
legend('Standard','Noise','Ref','Location','southeast')
grid on

subplot(3,1,3)
plot(time, z_std, 'LineWidth',linewidth)
hold on
plot(time, z_nl, 'LineWidth',linewidth)
hold on
plot(time, r(3,:), '--', 'LineWidth',linewidth)
ylabel('Position Z [m]') 
xlabel('Time [s]')
xlim([0 8])
ylim([-2.2 -0.8])
legend('Standard','Noise','Ref','Location','northeast')
grid on

% Euler Angles
myFigure7= figure();

subplot(3,1,1)
plot(time, roll_std, 'LineWidth',linewidth)
hold on
plot(time, roll_nl, 'LineWidth',linewidth)
ylabel('\phi [\circ]') 
legend('Standard','Noise','Location','northeast')
grid on
xlim([0 8])

subplot(3,1,2)
plot(time, pitch_std, 'LineWidth',linewidth)
hold on
plot(time, pitch_nl, 'LineWidth',linewidth)
ylabel('\theta [\circ]') 
legend('Standard','Noise','Location','southeast')
grid on
xlim([0 8])

subplot(3,1,3)
plot(time, yaw_std, 'LineWidth',linewidth)
hold on
plot(time, yaw_nl, 'LineWidth',linewidth)
ylabel('\psi [\circ]') 
xlabel('Time [s]')
legend('Standard','Noise','Location','southeast')
grid on
xlim([0 8])
ylim([-2.5 2.5])

saveFigAsPDF(myFigure6,'Figures_PDF/Position_noise_stepXYZ_model2');
saveFigAsPDF(myFigure7,'Figures_PDF/Orientation_noise_stepXYZ_model2');

