function output = DeePC_Optimizer_osqp(input)

%% This function is used in Simulink as interpreted Matlab function

%% GLOBAL VARIABLES
global start_flag uini yini

%% PERSISTENT VARIABLES
% Parameters: Get from workspace
persistent param exc_param DeePC_param DeePC_param_perf Tini lambda2_g lambda2_s...
    osqp_model osqp_model_perf

if isempty(param) || start_flag
    param = evalin('base', 'param');
    exc_param = evalin('base', 'exc_param');
    DeePC_param = evalin('base', 'DeePC_param');
    DeePC_param_perf = evalin('base', 'DeePC_param_perf');
    Tini = evalin('base', 'Tini');
    lambda2_g = evalin('base', 'lambda2_g');
    lambda2_s = evalin('base', 'lambda2_s');
    osqp_model = evalin('base', 'osqp_model');
    osqp_model_perf = evalin('base', 'osqp_model_perf');
    
    disp('Initialization Successful');
    start_flag = false;
end

%% INPUT VARIABLES: r_in
% r_in: Reference input [R^Nref]
r_in = input;

r = zeros(param.p, 1);
for i = 1 : DeePC_param.Nref
    j = DeePC_param.ref_outputs(i);
    r(j) = r_in(i);
end
r_vec = repmat(r, exc_param.Nd, 1);

%% Steady state trajectory mapper
% Only perform if we are not optimizing over steady state gs already
if ~param.opt_steady_state
    A_gs = [DeePC_param.U_p; DeePC_param.U_f; DeePC_param.Y_p; DeePC_param.Y_f];
    b_gs = [repmat(DeePC_param.us, Tini + exc_param.Nd, 1); repmat(r, Tini + exc_param.Nd + 1, 1)];
    gs =  A_gs \ b_gs;
    %gs =  zeros(DeePC_param.Ng, 1);

    if param.compare_to_perf
        A_gs_perf = [DeePC_param_perf.U_p; DeePC_param_perf.U_f; DeePC_param_perf.Y_p; DeePC_param_perf.Y_f];
        b_gs_perf = [repmat(DeePC_param_perf.us, Tini + exc_param.Nd, 1); repmat(r, Tini + exc_param.Nd + 1, 1)];
        gs_perf =  A_gs_perf \ b_gs_perf;
    end
end

%% Linear cost update
if param.setup_sparse_opt
    if param.opt_steady_state
        q_g = zeros(DeePC_param.Ng, 1);
    else
        q_g = -lambda2_g * DeePC_param.Wg * gs;
    end
    q_s = zeros(DeePC_param.Ns, 1);
    q_uf = DeePC_param.q_uf;
    q_yf = -kron(eye(exc_param.Nd), DeePC_param.Q) * r_vec;
    q_yt = -DeePC_param.P * r;
    q = [q_g; q_s; q_uf; q_yf; q_yt];
    if param.opt_steady_state
        q_gs = zeros(DeePC_param.Ng, 1);
        q_us = zeros(param.m, 1);
        q = [q; q_gs; q_us];
    end
else
    q_g = DeePC_param.q_g...
        - DeePC_param.Y_f(1:end-param.p,:).' * kron(eye(exc_param.Nd), DeePC_param.Q) * r_vec...
        - DeePC_param.Y_f(end-param.p+1:end,:).' * DeePC_param.P * r...
        - lambda2_g * DeePC_param.Wg * gs...
        - lambda2_s * DeePC_param.Y_p.' * yini;
    q = q_g;
end

if param.compare_to_perf
    if param.setup_sparse_opt
        if param.opt_steady_state
            q_g_perf = zeros(DeePC_param_perf.Ng, 1);
        else
            q_g_perf = -0 * DeePC_param_perf.Wg * gs_perf;
        end
        q_s_perf = zeros(DeePC_param_perf.Ns, 1);
        q_uf_perf = DeePC_param_perf.q_uf;
        q_yf_perf = -kron(eye(exc_param.Nd), DeePC_param_perf.Q) * r_vec;
        q_yt_perf = -DeePC_param_perf.P * r;
        q_perf = [q_g_perf; q_s_perf; q_uf_perf; q_yf_perf; q_yt_perf];
        if param.opt_steady_state
            q_gs_perf = zeros(DeePC_param_perf.Ng, 1);
            q_us_perf = zeros(param.m, 1);
            q_perf = [q_perf; q_gs_perf; q_us_perf];
        end
    else
        q_g_perf = DeePC_param_perf.q_g...
            - DeePC_param_perf.Y_f(1:end-param.p,:).' * kron(eye(exc_param.Nd), DeePC_param_perf.Q) * r_vec...
            - DeePC_param_perf.Y_f(end-param.p+1:end,:).' * DeePC_param_perf.P * r...
            - 0 * DeePC_param_perf.Wg * gs_perf...
            - lambda2_s * DeePC_param_perf.Y_p.' * yini;
        q_perf = q_g_perf;
    end
end

%% Equality constraint update
if param.setup_sparse_opt
    l_eq = [DeePC_param.l_eq; uini; yini];
    if param.opt_steady_state
        l_eq = [l_eq; repmat(r, Tini + exc_param.Nd + 1, 1)];
    end
else
    l_eq = uini;
end
u_eq = l_eq;
l = [DeePC_param.l_in; l_eq];
u = [DeePC_param.u_in; u_eq];

if param.compare_to_perf
    l_perf = [DeePC_param_perf.l_in; l_eq];
    u_perf = [DeePC_param_perf.u_in; u_eq];
end

%% Update and solve model
osqp_model.update('q', q, 'l', l, 'u', u);

if param.compare_to_perf
    osqp_model_perf.update('q', q_perf, 'l', l_perf, 'u', u_perf);
end

%% Solve model
osqp_result = osqp_model.solve();

if param.compare_to_perf
    % It was found necessary to pause with non-sparse formulation -
    % otherwise perfect output is high WEIRD
    if ~param.setup_sparse_opt
        pause(0.2);
    end
    osqp_result_perf = osqp_model_perf.solve();
end

%% OUTPUT VARIABLES: [g; sigma; cost; g_perf; sigma_perf; cost_perf; u]:
% g: Trajectory mapper [R^Ng]
% sigma: Slack on yini [R^(p * Tini)]
% cost: Optimization cost [R^1]
% solve_time: Optimization solve time [R^1]
% g_perf: Trajectory mapper using 'perfect' data [R^Ng]
% sigma_perf: Slack on yini using 'perfect' data [R^(p * Tini)]
% cost_perf: Optimization cost using 'perfect' data [R^1]
% solve_time_perf: Optimization solve time using 'perfect' data [R^1]
% u: Input to apply
if osqp_result.info.status_val > 0
    output = osqp_result.x(1:DeePC_param.Ng);
    if param.setup_sparse_opt
        output = [output;...
            osqp_result.x(DeePC_param.Ng+1:DeePC_param.Ng+DeePC_param.Ns)];
    else
        output = [output;...
            DeePC_param.Y_p * output - yini];
    end
    output = [output;...
        osqp_result.info.obj_val; osqp_result.info.run_time];
else
    output = zeros(DeePC_param.Ng + DeePC_param.Ns + 2, 1);
end

if param.compare_to_perf && osqp_result_perf.info.status_val > 0
    output_perf = osqp_result_perf.x(1:DeePC_param_perf.Ng);
    if param.setup_sparse_opt
        output_perf = [output_perf;...
            osqp_result_perf.x(DeePC_param_perf.Ng+1:DeePC_param_perf.Ng+DeePC_param_perf.Ns)];
    else
        output_perf = [output_perf;...
            DeePC_param_perf.Y_p * output_perf - yini];
    end
    output_perf = [output_perf;...
        osqp_result_perf.info.obj_val; osqp_result_perf.info.run_time];
else
    output_perf = zeros(length(output), 1);
end

%% Control input to apply
% U_f(1:m,:)*g gets first input in optimal input sequence
u = DeePC_param.U_f(1:param.m,:) * output(1:DeePC_param.Ng);
% Augment with 0 if DeePC is not controlling yaw
if ~param.DeePC_yaw_control
    u = [u; 0];
end

output = [output; output_perf; u];

end