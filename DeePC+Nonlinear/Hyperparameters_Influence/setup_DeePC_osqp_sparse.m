function [DeePC_param, osqp_model] = setup_DeePC_osqp_sparse(param, exc_param,...
    u_data, y_data, Tini, lambda2_g, lambda2_s,...
    Q_x, Q_y, Q_z, Q_yaw, R_thrust, R_roll, R_pitch, R_yaw,...
    P_x, P_x_pitch, P_y, P_y_roll, P_z, P_roll, P_pitch, P_yaw)

%% DECISION VARIABLES: [g; sigma; u_f; y_f; y_t]
% g: Trajectory mapper [U_p; Y_p; U_f; Y_f] * g = [uini; yini; u_f; y_f; y_t] [R^Ng]
% sigma: Slack variable on yini [R^(p * Tini)]
% u_f: Future inputs [R^(m * Nd)]
% y_f: Future outputs [R^(p * Nd)]
% y_t: Terminal future output [R^p]

%% Hankel matrices
H_u = data2hankel(u_data, Tini + exc_param.Nd + 1);
H_y = data2hankel(y_data, Tini + exc_param.Nd + 1);
DeePC_param.U_p = H_u(1:param.m*Tini,:);
DeePC_param.U_f = H_u(param.m*Tini+1:end-param.m,:);
DeePC_param.Y_p = H_y(1:param.p*Tini,:);
DeePC_param.Y_f = H_y(param.p*Tini+1:end,:);

%% SVD Weighing
DeePC_param.Ng = size(DeePC_param.U_p, 2);
[~, ~, DeePC_param.V] = svd(H_y);

DeePC_param.Wg = eye(DeePC_param.Ng);


%% Reference outputs
% Reference is x, y, z, yaw
% If DeePC yaw control is disabled, do not track yaw
if param.DeePC_yaw_control
    if param.measure_angles
        DeePC_param.ref_outputs = [1; 2; 3; 6];
    else
        DeePC_param.ref_outputs = [1; 2; 3; 4];
    end
    DeePC_param.ref_states = [1; 2; 3; 9];
else
    DeePC_param.ref_outputs = [1; 2; 3];
    DeePC_param.ref_states = [1; 2; 3];
end
DeePC_param.Nref = length(DeePC_param.ref_outputs);

%% Cost matrices
DeePC_param.Q = zeros(param.p); % Output cost matrix
DeePC_param.Q(1,1) = Q_x;
DeePC_param.Q(2,2) = Q_y;
DeePC_param.Q(3,3) = Q_z;
DeePC_param.R = zeros(param.m); % Input cost matrix
DeePC_param.R(1,1) = R_thrust;
DeePC_param.R(2,2) = R_roll;
DeePC_param.R(3,3) = R_pitch;
DeePC_param.P = zeros(param.p); % Terminal cost matrix
DeePC_param.P(1,1) = P_x;
DeePC_param.P(2,2) = P_y;
DeePC_param.P(3,3) = P_z;
if param.measure_angles
    DeePC_param.P(1,5) = P_x_pitch;
    DeePC_param.P(2,4) = P_y_roll;
    DeePC_param.P(4,2) = P_y_roll;
    DeePC_param.P(4,4) = P_roll;
    DeePC_param.P(5,1) = P_x_pitch;
    DeePC_param.P(5,5) = P_pitch;
end
if param.DeePC_yaw_control
    DeePC_param.Q(end,end) = Q_yaw;
    DeePC_param.R(end,end) = R_yaw;
    DeePC_param.P(end,end) = P_yaw;
end

%% Quadratic cost matrix in style of OSQP
DeePC_param.NuIni = param.m * Tini;
DeePC_param.NyIni = param.p * Tini;
Nu = param.m * exc_param.Nd;
Ny = param.p * exc_param.Nd;
DeePC_param.Ns = DeePC_param.NyIni;
P_g = lambda2_g * DeePC_param.Wg;
P_s = lambda2_s * eye(DeePC_param.Ns);
P_uf = kron(eye(exc_param.Nd), DeePC_param.R);
P_yf = kron(eye(exc_param.Nd), DeePC_param.Q);
P_yt = DeePC_param.P;
P = blkdiag(P_g, P_s, P_uf, P_yf, P_yt);
P_min_eig = min(eig(P));

%% Linear cost vector in style of OSQP
if param.DeePC_yaw_control
    DeePC_param.us = [param.nrotor_vehicle_mass_for_controller * param.g;...
        0; 0; 0];
else
    DeePC_param.us = [param.nrotor_vehicle_mass_for_controller * param.g;...
        0; 0];
end
us_vec = repmat(DeePC_param.us, exc_param.Nd, 1);

DeePC_param.q_uf = -kron(eye(exc_param.Nd), DeePC_param.R) * us_vec;
q_s = zeros(DeePC_param.Ns, 1);

% Temporary placeholders - these will be updated in runtime
q_g = zeros(DeePC_param.Ng, 1);
q_yf = zeros(Ny, 1);
q_yt = zeros(param.p, 1);
q = [q_g; q_s; DeePC_param.q_uf; q_yf; q_yt];

%% Inequality constraints in style of OSQP
A_in = [zeros(Nu, DeePC_param.Ng + DeePC_param.Ns) eye(Nu) zeros(Nu, Ny + param.p)];
A_in = [A_in; zeros(Ny + param.p, DeePC_param.Ng + DeePC_param.Ns + Nu) eye(Ny + param.p)];

DeePC_param.l_in = repmat(param.input_min, exc_param.Nd, 1);
DeePC_param.l_in = [DeePC_param.l_in; repmat(param.output_min, exc_param.Nd + 1, 1)];
DeePC_param.u_in = repmat(param.input_max, exc_param.Nd, 1);
DeePC_param.u_in = [DeePC_param.u_in; repmat(param.output_max, exc_param.Nd + 1, 1)];

%% Equality constraints in style of OSQP
A_eq = [DeePC_param.U_f zeros(Nu, DeePC_param.Ns) -eye(Nu) zeros(Nu, Ny + param.p)];
A_eq = [A_eq; DeePC_param.Y_f zeros(Ny + param.p, DeePC_param.Ns + Nu) -eye(Ny + param.p)];
A_eq = [A_eq; DeePC_param.U_p zeros(DeePC_param.NuIni, DeePC_param.Ns + Nu + Ny + param.p)];
A_eq = [A_eq; DeePC_param.Y_p -eye(DeePC_param.Ns) zeros(DeePC_param.Ns, Nu + Ny + param.p)];

DeePC_param.l_eq = zeros(Nu + Ny + param.p, 1);

% Temporary placeholders - these will be updated in runtime
l_eq = [DeePC_param.l_eq; zeros(DeePC_param.NuIni + DeePC_param.NyIni, 1)];
u_eq = l_eq;

A = [A_in; A_eq];
l = [DeePC_param.l_in; l_eq];
u = [DeePC_param.u_in; u_eq];

%% OSQP model setup
osqp_model = osqp;
settings = osqp_model.default_settings();
settings.verbose = param.verbose;
osqp_model.setup(sparse(P), q, sparse(A), l, u, settings);

% Dummy solve model once to ensure first runtime does not capture setup
% time
osqp_model.solve();

end