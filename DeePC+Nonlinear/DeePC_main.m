clear;
close all;
clc;

%% Load parameters
param = load_crazyflie_parameters();

flag_input_transform_nonlinearity = true;
flag_noise = 1;
wave_time = 10;

% run_time = 30;

run_time = 10;

start_time = 2;

step_x = true;
step_y = true;
step_z = true;
amp_step = 1;

K_roll = param.K_lqr_outer_loop(2,7);
K_pitch = param.K_lqr_outer_loop(3,8);
K_yaw = param.K_lqr_outer_loop(4,9);

Kp_xy = - param.K_lqr_outer_loop(3,1)/param.K_lqr_outer_loop(2,7);
Kv_xy = - param.K_lqr_outer_loop(3,4)/param.K_lqr_outer_loop(2,7);
Kp_z = - param.K_lqr_outer_loop(1,3);
Kv_z = - param.K_lqr_outer_loop(1,6);
Ki_xy = 0;
Ki_z = 0;

mass = param.nrotor_vehicle_mass_for_controller;
accel_g = param.g;

M = [ones(1,4);param.nrotor_vehicle_layout_true];
inv_M = inv(M);

%% Set initial condition prior to data collection
param.nrotor_initial_condition(1:3) = [0.0; 0.0; -1.0];

%% Turn noise on/off prior to data collection
param.measurement_noise_full_state = 1;
param.measurement_noise_body_rates = 1;
param.measurement_noise_body_accelerations = 1;

%% Collect data
% Main tuning parmeters

% Additional data points over persistence of excitation
Td = 900;

% Tini
Tini = 5;

% rng_seed: Random seed used for random excitation signal generation (not
% used for PRBS)
rng_seed = 0;

% Generate excitation signals
exc_param = generate_excitation_signal(param, rng_seed, Td, Tini);


[u_data, y_data] = collect_DeePC_data(param, exc_param, Tini);

fprintf('Data collected\n\n');

% filename = "Saved_Results\PRBS\gazebo.mat";
% save(filename,"param","exc_param","u_data","y_data");

%% Setup DeePC optimization
% Main tuning parameters

% Quadratic state cost matrix coefficients
Q_x = param.Q(1,1);
Q_y = param.Q(2,2);
Q_z = param.Q(3,3);
Q_yaw = param.Q(end,end);

% Quadratic input cost matrix coefficients
R_thrust = param.R(1,1);
R_roll = param.R(2,2);
R_pitch = param.R(3,3);
R_yaw = param.R(4,4);

% Quadratic state cost matrix coefficients
P_x = 0;
P_x_pitch = 0;
P_y = 0;
P_y_roll = 0;
P_z = 0;
P_roll = 0;
P_pitch = 0;
P_yaw = 0;

% 2-norm penalty on g
lambda2_g = 500;

% 2-norm penalty on yini slack
lambda2_s = 7.5e8;

[DeePC_param, osqp_model]...
    = setup_DeePC_osqp_sparse(param, exc_param, u_data, y_data,...
    Tini, lambda2_g, lambda2_s,...
    Q_x, Q_y, Q_z, Q_yaw, R_thrust, R_roll, R_pitch, R_yaw,...
    P_x, P_x_pitch, P_y, P_y_roll, P_z, P_roll, P_pitch, P_yaw);

us_out = 0;
DeePC_param_perf = [];    
osqp_model_perf = [];
        

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
% Initialize uini yini
global start_flag uini yini
start_flag = true;
uini = zeros(DeePC_param.NuIni, 1);
yini = zeros(DeePC_param.NyIni, 1);

% start_time = max([5 Tini*param.sample_time_controller_outer]);


sim_time = start_time + run_time;

sim('Iris_DeePC_Control_Gazebo');


fprintf("Done running DeePC\n\n");

%% Plots

plot_aux_DeePC(param, total_thrust_cmd, body_rates_cmd,...
    full_state_act, full_state_meas, r_out, run_time,...
    start_time);

% fg = plot_DeePC(param, exc_param, DeePC_param, DeePC_param_perf,...
%     total_thrust_cmd, body_rates_cmd, full_state_act, full_state_meas,...
%     r_out, g_out, sigma_out, solve_time_out,...
%     g_perf_out, solve_time_perf_out,...
%     Tini);

 %% Save Results
% filename = "C:\Users\simao\OneDrive\Ambiente de Trabalho\GitHub\Thesis\DeePC_tests_z_down\Saved_Results\Control_Methods_Comparison\DeePC_yaw_bias_25.mat";
% 
% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","Tini","run_time","solve_time_out");

% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","Tini","run_time","solve_time_out",...
%     "total_thrust_collected","total_thrust_PRBS","total_thrust_LQR",...
%     "eulerd_collected","eulerd_PRBS","eulerd_LQR","exc_param","u_data","y_data");

% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","Tini","run_time","solve_time_out",...
%     "exc_param","u_data","y_data");
