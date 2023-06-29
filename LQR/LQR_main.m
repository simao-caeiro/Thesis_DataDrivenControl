clear;
close all;
clc;

%% Load parameters
param = load_crazyflie_parameters();

flag_input_transform_nonlinearity = false;
flag_noise = 1;
wave_time = 10;

% run_time = 30;

run_time = 10;

start_time = 2;

step_x = true;
step_y = true;
step_z = true;
amp_step = 1;

% K_roll = param.K_lqr_outer_loop(2,7);
% K_pitch = param.K_lqr_outer_loop(3,8);
% K_yaw = param.K_lqr_outer_loop(4,9);

K_roll = 4.583;
K_pitch = 4.583;
K_yaw = 1.375;

% K_roll = 4.583*0.25;
% K_pitch = 4.583*0.25;
% K_yaw = 1.375*0.25;

Kp_xy = - param.K_lqr_outer_loop(3,1)/param.K_lqr_outer_loop(2,7);
Kv_xy = - param.K_lqr_outer_loop(3,4)/param.K_lqr_outer_loop(2,7);
% Kv_xy = 0.28;
Kp_z = - param.K_lqr_outer_loop(1,3);
Kv_z = - param.K_lqr_outer_loop(1,6);
% Kv_z = 7.5;
Ki_xy = 0;
Ki_z = 0; % VER!!

mass = param.nrotor_vehicle_mass_for_controller;
accel_g = param.g;

M = [ones(1,4);param.nrotor_vehicle_layout_true];
inv_M = inv(M);

%%
% value_n = 1;
% value_x = 0.25;
% value_y = 0.25;
% value_z = 0.25;
% 
% param.pid_x_body_rate_kp = param.pid_x_body_rate_kp*value_x;
% param.pid_x_body_rate_ki = param.pid_x_body_rate_ki*value_n;
% param.pid_x_body_rate_kd = param.pid_x_body_rate_kd*value_n;
% 
% param.pid_y_body_rate_kp = param.pid_x_body_rate_kp*value_y;
% param.pid_y_body_rate_ki = param.pid_x_body_rate_ki*value_n;
% param.pid_y_body_rate_kd = param.pid_x_body_rate_kd*value_n;
% 
% param.pid_y_body_rate_kp = param.pid_x_body_rate_kp*value_z;
% param.pid_y_body_rate_ki = param.pid_x_body_rate_ki*value_n;
% param.pid_y_body_rate_kd = param.pid_x_body_rate_kd*value_n;


%% Turn noise on/off prior to running
param.measurement_noise_full_state = flag_noise;
param.measurement_noise_body_rates = flag_noise;
param.measurement_noise_body_accelerations = flag_noise;


%% Set initial condition prior to running
param.nrotor_initial_condition(1:3) = [1; 1; -1];
% param.nrotor_initial_condition(1:3) = [0; 0; 0];
param.nrotor_final_condition = param.nrotor_initial_condition;

if step_x
    param.nrotor_final_condition(1) = param.nrotor_final_condition(1) + amp_step;
end

if step_y
    param.nrotor_final_condition(2) = param.nrotor_final_condition(2) + amp_step;
end

if step_z
    param.nrotor_final_condition(3) = param.nrotor_final_condition(3) - amp_step;
end

%% Run DeePC simulation

sim_time = start_time + run_time;

sim('Iris_LQR_Control_Gazebo');


fprintf("Done running DeePC\n\n");

%% Plots

plot_aux_LQR(param, total_thrust_cmd, body_rates_cmd,...
    full_state_act, full_state_meas, r_out, run_time,...
    start_time);

 %% Save Results
% filename = "C:\Users\simao\OneDrive\Ambiente de Trabalho\GitHub\Thesis\DeePC_tests_z_down\Saved_Results\Control_Methods_Comparison\LQR_yaw_bias_25.mat";
% 
% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","run_time");

% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","Tini","run_time","solve_time_out",...
%     "total_thrust_collected","total_thrust_PRBS","total_thrust_LQR",...
%     "eulerd_collected","eulerd_PRBS","eulerd_LQR","exc_param","u_data","y_data");
