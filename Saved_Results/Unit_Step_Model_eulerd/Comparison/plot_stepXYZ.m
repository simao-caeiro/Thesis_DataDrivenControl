clear;
close all;
clc;

%% Load variables: Nonlinear controller
load("Model1_stepXYZ.mat")

% Common variables
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;

start_time = 2;
sim_time = start_time + run_time;

time = 0:param.sample_time_measurements_full_state:sim_time;

% Specific variables
u_model1 = [total_thrust_cmd body_rates_cmd].';

x_state_model1 = full_state_act(:,1:9).';

x_model1 = x_state_model1(1,:);
y_model1 = x_state_model1(2,:);
z_model1 = x_state_model1(3,:);

roll_model1 = x_state_model1(7,:)*180/pi;
pitch_model1 = x_state_model1(8,:)*180/pi;
yaw_model1 = x_state_model1(9,:)*180/pi;

time_comp_model1 = solve_time_out;

%% Load variables: Linear controller
load("Model2_stepXYZ.mat")

% Specific variables
u_model2 = [total_thrust_cmd body_rates_cmd].';

x_state_model2 = full_state_act(:,1:9).';

x_model2 = x_state_model2(1,:);
y_model2= x_state_model2(2,:);
z_model2 = x_state_model2(3,:);

roll_model2 = x_state_model2(7,:)*180/pi;
pitch_model2 = x_state_model2(8,:)*180/pi;
yaw_model2 = x_state_model2(9,:)*180/pi;

time_comp_model2 = solve_time_out;

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
% plot(time, x_l, 'LineWidth',1.5)
% hold on
% plot(time, y_nl, 'LineWidth',1.5)
% hold on
% plot(time, y_l, 'LineWidth',1.5)
% hold on
% plot(time, r(1,:), '--', 'LineWidth',1.5)
% ylabel('Position [m]') 
% xlabel('Time [s]')
% legend('X_{nonlinear}','X_{linear}','Y_{nonlinear}','Y_{linear}','Ref','Location','east')
% grid on
% xlim([0 8])
% 
% % Position Z
% myFigure2 = figure();
% plot(time, z_nl, 'LineWidth',1.5)
% hold on
% plot(time, z_l, 'LineWidth',1.5)
% hold on
% plot(time, r(3,:), '--', 'LineWidth',1.5)
% ylabel('Position Z [m]') 
% xlabel('Time [s]')
% legend('Nonlinear','Linear','ref','Location','best')
% grid on
% xlim([0 8])
% 
% % Euler Angles
% myFigure3 = figure();
% plot(time, roll_nl, 'LineWidth',1.5)
% hold on
% plot(time, roll_l, 'LineWidth',1.5)
% hold on
% plot(time, pitch_nl, 'LineWidth',1.5)
% hold on
% plot(time, pitch_l, 'LineWidth',1.5)
% hold on
% plot(time, yaw_nl, 'LineWidth',1.5)
% hold on
% plot(time, yaw_l, 'LineWidth',1.5)
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


% Position XYZ
myFigure4= figure();

subplot(3,1,1)
plot(time, x_model1, 'LineWidth',1.5)
hold on
plot(time, x_model2, 'LineWidth',1.5)
hold on
plot(time, r(1,:), '--', 'LineWidth',1.5)
ylabel('Position X [m]') 
xlim([0 7])
ylim([0.5 2.5])
legend('Model A','Model B','Ref','Location','best')
grid on

subplot(3,1,2)
plot(time, y_model1, 'LineWidth',1.5)
hold on
plot(time, y_model2, 'LineWidth',1.5)
hold on
plot(time, r(2,:), '--', 'LineWidth',1.5)
ylabel('Position Y [m]') 
xlim([0 7])
ylim([0.5 2.5])
legend('Model A','Model B','Ref','Location','best')
grid on

subplot(3,1,3)
plot(time, z_model1, 'LineWidth',1.5)
hold on
plot(time, z_model2, 'LineWidth',1.5)
hold on
plot(time, r(3,:), '--', 'LineWidth',1.5)
ylabel('Position Z [m]') 
xlabel('Time [s]')
xlim([0 7])
ylim([-2.5 -0.5])
legend('Model A','Model B','Ref','Location','best')
grid on

% Euler Angles
myFigure5= figure();

subplot(3,1,1)
plot(time, roll_model1, 'LineWidth',1.5)
hold on
plot(time, roll_model2, 'LineWidth',1.5)
ylabel('\phi [\circ]') 
legend('Model A','Model B','Location','northeast')
grid on
xlim([0 7])

subplot(3,1,2)
plot(time, pitch_model1, 'LineWidth',1.5)
hold on
plot(time, pitch_model2, 'LineWidth',1.5)
ylabel('\theta [\circ]') 
legend('Model A','Model B','Location','southeast')
grid on
xlim([0 7])

subplot(3,1,3)
plot(time, yaw_model1, 'LineWidth',1.5)
hold on
plot(time, yaw_model2, 'LineWidth',1.5)
ylabel('\psi [\circ]') 
xlabel('Time [s]')
legend('Model A','Model B','Location','northeast')
grid on
xlim([0 7])
ylim([-2 2])

% xlimit = 5/0.04;

% myFigure6 = figure();
% plot(time_comp_model1,'LineWidth',1.5)
% hold on
% plot(time_comp_model2,'LineWidth',1.5)
% ylabel('Solve Time [s]') 
% xlabel('Number of Time Samples')
% xlim([0 xlimit])
% % ylim([0.5 2.5])
% grid on

% saveFigAsPDF(myFigure4,'Figures_PDF/Position_stepXYZ_model1_vs_model2');
% saveFigAsPDF(myFigure5,'Figures_PDF/Orientation_stepXYZ_model1_vs_model2');
