function plot_aux_DeePC(param, test_run_total_thrust_cmd, test_run_body_rates_cmd,...
    test_run_full_state_act, test_run_full_state_meas, r_out, DeePC_run_time,...
    start_time)

% close all

% Organize data for plotting
u = [test_run_total_thrust_cmd test_run_body_rates_cmd].';
T_eq = param.nrotor_vehicle_mass_true*param.g;

x = test_run_full_state_act(:,1:9).';
x_meas = test_run_full_state_meas(:,1:9).';

x_pos = x(1,:);
x_pos_meas = x_meas(1,:);

y_pos = x(2,:);
y_pos_meas = x_meas(2,:);

z_pos = x(3,:);
z_pos_meas = x_meas(3,:);

% vel_x_pos = x(4,:);
% vel_x_pos_meas = x_meas(4,:);
% 
% vel_y_pos = x(5,:);
% vel_y_pos_meas = x_meas(5,:);
% 
% vel_z_pos = x(6,:);
% vel_z_pos_meas = x_meas(6,:);

roll = x(7,:)*180/pi;
roll_meas = x_meas(7,:)*180/pi;

pitch = x(8,:)*180/pi;
pitch_meas = x_meas(8,:)*180/pi;

yaw = x(9,:)*180/pi;
yaw_meas = x_meas(9,:)*180/pi;

r = r_out.';

sim_time = start_time + DeePC_run_time;

time = 0:param.sample_time_measurements_full_state:sim_time;

% % Body-rates
% figure()
% plot(time, u(2,:))
% hold on
% plot(time, u(3,:))
% hold on
% plot(time, u(4,:))
% hold on
% ylabel('\omega [rad/s]') 
% xlabel('Time [s]')
% legend('\omega_x','\omega_y','\omega_z','Location','best')
% 
% % Thrust
% figure()
% plot(time, u(1,:))
% hold on
% plot(time, T_eq*ones(1,length(time)), '--')
% ylabel('T [N]') 
% xlabel('Time [s]')
% legend('','ref','Location','best')

% % Yaw
% figure()
% plot(time, yaw)
% hold on
% plot(time, yaw_meas)
% ylabel('Psi [º]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% % Pitch
% figure()
% plot(time, pitch)
% hold on
% plot(time, pitch_meas)
% ylabel('Theta [º]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% % Roll
% figure()
% plot(time, roll)
% hold on
% plot(time, roll_meas)
% ylabel('Phi [º]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')

% % Velocity Z
% figure()
% plot(time, vel_z_pos)
% hold on
% plot(time, vel_z_pos_meas)
% ylabel('Vel Z [m/s]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% % Velocity Y
% figure()
% plot(time, vel_y_pos)
% hold on
% plot(time, vel_y_pos_meas)
% ylabel('Vel Y [m/s]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% % Velocity X
% figure()
% plot(time, vel_x_pos)
% hold on
% plot(time, vel_x_pos_meas)
% ylabel('Vel X [m/s]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')

% % Position Z
% figure()
% plot(time, z_pos)
% hold on
% plot(time, z_pos_meas)
% hold on
% plot(time, r(3,:))
% ylabel('Z [m]') 
% xlabel('Time [s]')
% legend('actual','measured','ref','Location','best')
% 
% % Position Y
% figure()
% plot(time, y_pos)
% hold on
% plot(time, y_pos_meas)
% hold on
% plot(time, r(2,:))
% ylabel('Y [m]') 
% xlabel('Time [s]')
% legend('actual','measured','ref','Location','best')
% 
% % Position X
% figure()
% plot(time, x_pos)
% hold on
% plot(time, x_pos_meas)
% hold on
% plot(time, r(1,:))
% ylabel('X [m]') 
% xlabel('Time [s]')
% legend('actual','measured','ref','Location','best')

% Inputs
f_inputs = figure();
f_inputs.Position = [50 100 1450 600];
t_input = tiledlayout(1,2,'TileSpacing','tight','Padding','tight');
title(t_input,'Inputs','FontWeight','bold')

nexttile
plot(time, u(1,:))
hold on
plot(time, T_eq*ones(1,length(time)), '--')
ylabel('T [N]') 
xlabel('Time [s]')
legend('','ref','Location','best')
grid on

nexttile
plot(time, u(2,:))
hold on
plot(time, u(3,:))
hold on
plot(time, u(4,:))
hold on
ylabel('\omega [rad/s]') 
xlabel('Time [s]')
legend('\omega_x','\omega_y','\omega_z','Location','best')
grid on


% Euler Angles
f_ang = figure();
f_ang.Position = [50 100 1450 600];
t_ang = tiledlayout(1,3,'TileSpacing','tight','Padding','tight');
title(t_ang,'Euler Angles','FontWeight','bold')

nexttile
plot(time, roll)
hold on
plot(time, roll_meas)
ylabel('Phi [º]') 
xlabel('Time [s]')
legend('actual','measured','Location','best')
grid on

nexttile
plot(time, pitch)
hold on
plot(time, pitch_meas)
ylabel('Theta [º]') 
xlabel('Time [s]')
legend('actual','measured','Location','best')
grid on

nexttile
plot(time, yaw)
hold on
plot(time, yaw_meas)
ylabel('Psi [º]') 
xlabel('Time [s]')
legend('actual','measured','Location','best')
grid on


% % Velocities
% f_vel = figure();
% f_vel.Position = [50 100 1450 600];
% t_vel = tiledlayout(1,3,'TileSpacing','tight','Padding','tight');
% title(t_vel,'Velocities','FontWeight','bold')
% 
% nexttile
% plot(time, vel_x_pos)
% hold on
% plot(time, vel_x_pos_meas)
% ylabel('Vel X [m]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% nexttile
% plot(time, vel_y_pos)
% hold on
% plot(time, vel_y_pos_meas)
% ylabel('Vel Y [m]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')
% 
% nexttile
% plot(time, vel_z_pos)
% hold on
% plot(time, vel_z_pos_meas)
% ylabel('Vel Z [m]') 
% xlabel('Time [s]')
% legend('actual','measured','Location','best')



% Positions
f_pos = figure();
f_pos.Position = [50 100 1450 600];
t_pos = tiledlayout(1,3,'TileSpacing','tight','Padding','tight');
title(t_pos,'Positions','FontWeight','bold')

nexttile
plot(time, x_pos)
hold on
plot(time, x_pos_meas)
hold on
plot(time, r(1,:))
ylabel('X [m]') 
xlabel('Time [s]')
legend('actual','measured','ref','Location','best')
% ylim([0.8 1.2])
grid on

nexttile
plot(time, y_pos)
hold on
plot(time, y_pos_meas)
hold on
plot(time, r(2,:))
ylabel('Y [m]') 
xlabel('Time [s]')
legend('actual','measured','ref','Location','best')
% ylim([0.8 1.2])
grid on

nexttile
plot(time, z_pos)
hold on
plot(time, z_pos_meas)
hold on
plot(time, r(3,:))
ylabel('Z [m]') 
xlabel('Time [s]')
legend('actual','measured','ref','Location','best')
% ylim([-1.2 -0.8])
grid on

% figure()
% plot(time, x_pos)
% hold on
% plot(time, x_pos_meas)
% hold on
% plot(time, r(1,:))
% hold on 
% plot(time, y_pos)
% hold on
% plot(time, y_pos_meas)
% hold on
% plot(time, z_pos)
% hold on
% plot(time, z_pos_meas)
% hold on
% plot(time, r(3,:))
% ylabel('X [m]') 
% xlabel('Time [s]')
% legend('actual','measured','ref','Location','best')
% % ylim([0.8 1.2])
% grid on


end