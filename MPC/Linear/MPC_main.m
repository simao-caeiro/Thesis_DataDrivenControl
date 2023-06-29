clear;
close all;
clc;

%% Load parameters
param = load_crazyflie_parameters();

flag_input_transform_nonlinearity = true;
flag_noise = 0;
wave_time = 10;

% run_time = 30;

run_time = 10;
nk = run_time/0.04;

start_time = 0;

step_x = true;
step_y = true;
step_z = true;
amp_step = 1;

K_roll = param.K_lqr_outer_loop(2,7);
K_pitch = param.K_lqr_outer_loop(3,8);
K_yaw = param.K_lqr_outer_loop(4,9);

mass = param.nrotor_vehicle_mass_for_controller;
accel_g = param.g;

M = [ones(1,4);param.nrotor_vehicle_layout_true];
inv_M = inv(M);


%% Collect data
% Main tuning parmeters

% Additional data points over persistence of excitation
% Td = 331;
% Td = 900;
% Td = 600;

% Tini
% Tini = 6;
% Tini = 5;

Nd = 50;

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

%% Setup MPC optimization
% Main tuning parameters

Ad_mpc = param.Ad_MPC;
Bd_mpc = param.Bd_MPC;
Cd_mpc = eye(9);

Q_mpc = diag([40, 40, 500, 0, 0, 0, 0, 0, 40]);
R_mpc = 10*diag([0.5, 20, 20, 20]);
P_mpc = Q_mpc;
N_mpc = Nd;
xd0 = param.nrotor_initial_condition(1:9);
nxd = size(Bd_mpc,1);
nu_mpc = size(Bd_mpc,2);
ny = size(Cd_mpc,1);
nx = nxd+ny;

ref = param.nrotor_final_condition(1:9);

% compute batch matrices
[F,G,Qb,Rb,H,Fd,Gd,Hd,~,~,~,Yb_mpc] = GetBatchXiMatrices(Ad_mpc,Bd_mpc,...
                                    Cd_mpc,N_mpc,P_mpc,Q_mpc,R_mpc,ref);
Fb = H*F;
Gb = H*G;
Fdb = Hd*Fd;
Gdb = Hd*Gd;
Rt = Gb'*Qb*Gb + Rb;
St = Gb'*Qb;
Ky_mpc = Rt^(-1)*St;
K_mpc = Rt^(-1)*St*Fb;

mpc_param.Ky = Ky_mpc;
mpc_param.K = K_mpc;
mpc_param.nu = nu_mpc;
mpc_param.Yb = Yb_mpc;
mpc_param.N = N_mpc;
mpc_param.Cd = Cd_mpc;

U_mpc = zeros(nu_mpc,nk);
Xd_mpc(:,1) = xd0;
Xd_mpc(:,2) = xd0;
k_aux_mpc = 2;


%% Run MPC simulation
Ad = param.Ad_MPC;
Bd = param.Bd_MPC;


% Initialize uini yini
global start_flag
start_flag = true;
% uini = zeros(DeePC_param.NuIni, 1);
% yini = zeros(DeePC_param.NyIni, 1);

% start_time = max([5 Tini*param.sample_time_controller_outer]);


sim_time = start_time + run_time;

sim('Iris_DeePC_Control_Gazebo_MPC_aux_linear');


fprintf("Done running MPC\n\n");

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
% filename = "C:\Users\simao\OneDrive\Ambiente de Trabalho\GitHub\Thesis\DeePC_tests_z_down\Saved_Results\Unit_Step_Model_eulerd\Nonlinear_noise.mat";
% 
% save(filename,"param","total_thrust_cmd","body_rates_cmd",...
%     "full_state_act","full_state_meas","r_out","Tini","run_time","solve_time_out");
