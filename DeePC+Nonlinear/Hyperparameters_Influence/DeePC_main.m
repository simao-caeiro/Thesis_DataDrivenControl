clear;
close all;
clc;

for i_aux=1:5
    clearvars -except i_aux
    close all;
    %% Load parameters
    param = load_crazyflie_parameters(i_aux);
    
    flag_input_transform_nonlinearity = true;
    
    % run_time = 65;
    run_time = 30;
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
%     Td_aux = [300;600;800;900;1000;1200;1500;2000]; % i_aux = 1:8
%     Td = Td_aux(i_aux);
    
    % Tini
    % Tini = 6;
%       Tini = i_aux+2; % i_aux = 1:8
    Tini = 5;

%         Nd_aux = [10;25;40;50;60;75;90;100]; % i_aux = 1:8
%         Nd = Nd_aux(i_aux);
    Nd = 50;

    % % exc_signal_select: 1 = random signal, 2 = PRBS
    % exc_signal_select = 2;

    % rng_seed: Random seed used for random excitation signal generation (not
    % used for PRBS)
    rng_seed = 0;
    
    % Generate excitation signals
    exc_param = generate_excitation_signal(param, rng_seed, Td, Tini, Nd, i_aux);
    
    
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
    % P_x = param.P(1,1);
    % P_x_pitch = param.P(1,8);
    % P_y = param.P(2,2);
    % P_y_roll = param.P(2,7);
    % P_z = param.P(3,3);
    % P_roll = param.P(7,7);
    % P_pitch = param.P(8,8);
    % P_yaw = param.P(9,9);
    P_x = 0;
    P_x_pitch = 0;
    P_y = 0;
    P_y_roll = 0;
    P_z = 0;
    P_roll = 0;
    P_pitch = 0;
    P_yaw = 0;
    
    % 2-norm penalty on g
    % lambda2_g = 5.0e2;
%     lambda2_g_aux = [0;100;300;500;700;1000;2000;5000]; % i_aux = 1:8
%     lambda2_g = lambda2_g_aux(i_aux);
    lambda2_g = 500;
    
    % 2-norm penalty on yini slack
    % lambda2_s = 7.5e8;
%     lambda2_s_aux = [0;1e5;1e8;7e8;7.5e8;8e8;1e9;1e10]; % i_aux = 1:8
%     lambda2_s = lambda2_s_aux(i_aux);
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
    param.measurement_noise_full_state = 0;
    param.measurement_noise_body_rates = 0;
    param.measurement_noise_body_accelerations = 0;
    
    % param.nrotor_vehicle_mass_for_controller = 1.37;
    % param.nrotor_vehicle_mass_true = 1.3;
    
    
    %% Set initial condition prior to running
    % param.nrotor_initial_condition(1:3) = [1; 1; -1];
    param.nrotor_initial_condition(1:3) = [0; 0; 0];
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
    
    %% Run LQR simulation
    % Initialize uini yini
    global start_flag uini yini
    start_flag = true;
    uini = zeros(DeePC_param.NuIni, 1);
    yini = zeros(DeePC_param.NyIni, 1);
    
    % start_time = max([5 Tini*param.sample_time_controller_outer]);
    start_time = 5;
    
    sim_time = start_time + run_time;
    
    sim('Iris_DeePC_Control_Gazebo_hyper');
    
    
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
    
    %% Metrics Calculation
    
    x = full_state_act(:,1:9).';
    signal_x = x(1,:);
    signal_y = x(2,:);
    signal_z = -x(3,:);
    
    ref = [param.nrotor_final_condition(1);param.nrotor_final_condition(2);...
        -param.nrotor_final_condition(3)];
    
    [ts, S, t_comp, error_track] = ...
        compute_metrics(signal_x, signal_y, signal_z, solve_time_out, ref,...
        param.sample_time_controller_outer, start_time);

    X = sprintf('Iteration %d\nts:%f\nS:%f\nerror_track:%f\n',i_aux,ts(4),S(4),error_track(4));
    disp(X)

    disp('--------------------');
    
     
    
     %% Save Results
    filename = "Saved_Results\Ampl_pitch_roll\"+i_aux+".mat";
    
    save(filename,"param","total_thrust_cmd","body_rates_cmd",...
        "full_state_act","full_state_meas","r_out","Tini","run_time", ...
        "ts","S","t_comp","error_track","exc_param");
end