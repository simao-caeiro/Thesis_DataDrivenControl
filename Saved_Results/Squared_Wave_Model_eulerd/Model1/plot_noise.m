clear;
close all;
clc;

%% Load variables: Nonlinear controller
load("Nonlinear.mat")

% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 5;
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

time_comp_nl = solve_time_out;

%% Load variables: Nonlinear+noise controller
load("Nonlinear_noise.mat")

% Specific variables
u_noise = [total_thrust_cmd body_rates_cmd].';

x_state_noise = full_state_meas(:,1:9).';

x_noise = x_state_noise(1,:);
y_noise= x_state_noise(2,:);
z_noise = x_state_noise(3,:);

roll_noise = x_state_noise(7,:)*180/pi;
pitch_noise = x_state_noise(8,:)*180/pi;
yaw_noise = x_state_noise(9,:)*180/pi;

time_comp_noise = solve_time_out;

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

linewidth = 0.5;

% Position XY
myFigure1 = figure();
plot(time, x_nl, 'LineWidth',linewidth)
hold on
plot(time, x_noise, 'LineWidth',linewidth)
hold on
plot(time, y_nl, 'LineWidth',linewidth)
hold on
plot(time, y_noise, 'LineWidth',linewidth)
hold on
plot(time, r(1,:), '--', 'LineWidth',linewidth)
ylabel('Position [m]') 
xlabel('Time [s]')
legend('X','X_{noise}','Y','Y_{noise}','Ref','Location','east')
grid on
xlim([0 35])

% Position Z
myFigure2 = figure();
plot(time, z_nl, 'LineWidth',linewidth)
hold on
plot(time, z_noise, 'LineWidth',linewidth)
hold on
plot(time, r(3,:), '--', 'LineWidth',linewidth)
ylabel('Position Z [m]') 
xlabel('Time [s]')
legend('Without Noise','With Noise','ref','Location','best')
grid on
xlim([0 35])

% % Euler Angles
% myFigure3 = figure();
% plot(time, roll_nl, 'LineWidth',1.5)
% hold on
% plot(time, roll_noise, 'LineWidth',1.5)
% hold on
% plot(time, pitch_nl, 'LineWidth',1.5)
% hold on
% plot(time, pitch_noise, 'LineWidth',1.5)
% hold on
% plot(time, yaw_nl, 'LineWidth',1.5)
% hold on
% plot(time, yaw_noise, 'LineWidth',1.5)
% ylabel('Euler Angles [\circ]') 
% xlabel('Time [s]')
% legend('\phi_{nonlinear}','\phi_{linear}','\theta_{nonlinear}','\theta_{linear}','\psi_{nonlinear}','\psi_{linear}','Location','southeast')
% grid on
% xlim([0 8])

% saveFigAsPDF(myFigure1,'Figures_PDF/XY_stepXYZ_model2');
% saveFigAsPDF(myFigure2,'Figures_PDF/Z_stepXYZ_model2');
% saveFigAsPDF(myFigure3,'Figures_PDF/Euler_stepXYZ_model2');


% figure()
% plot(x_nl,y_nl)
% ylabel('Y [m]') 
% xlabel('X [n]')
% xlim([0.5 2.5])
% ylim([0.5 2.5])
% grid on


% % Position XYZ
% figure()
% plot(time, x_nl)
% hold on
% plot(time, x_l)
% hold on
% plot(time, y_nl)
% hold on
% plot(time, y_l)
% hold on
% plot(time, z_nl)
% hold on
% plot(time, z_l)
% hold on
% plot(time, r(1,:), '--')
% hold on
% plot(time, r(2,:), '--')
% hold on
% plot(time, r(3,:), '--')
% ylabel('Y [m]') 
% xlabel('Time [s]')
% legend('X_{nonlinear}','X_{linear}','Y_{nonlinear}','Y_{linear}','Z_{nonlinear}','Z_{linear}',...
%     'X_{ref}','Y_{ref}','Z_{ref}','Location','best')
% grid on

