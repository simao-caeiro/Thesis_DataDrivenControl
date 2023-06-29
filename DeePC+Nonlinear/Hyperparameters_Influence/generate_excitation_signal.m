function exc_param = generate_excitation_signal(param, rng_seed, Td, Tini,Nd, i_aux)
%% SPECIFY EXCITATION SIGNAL PARAMETERS
% exc_signal_select: 1 = random signal, 2 = PRBS
exc_param.exc_sample_time = param.sample_time_controller_outer;
exc_start_time = 5; % Excitation start time in s
exc_param.exc_num_averages = 1;
rng(rng_seed);

% Nc = 1; % Prediction horizon in seconds
% Nc = 2; % Prediction horizon in seconds
% exc_param.Nd = round(Nc / exc_param.exc_sample_time); % Prediction horizon in samples (discrete)
exc_param.Nd = Nd; % Prediction horizon in samples (discrete)

% Check if Td satisfies persistency of excitation condition
min_num_data_pts = (param.m + 1) * (Tini + exc_param.Nd + 1 + param.n + param.d) - 1;
if Td < min_num_data_pts
    fprintf('WARNING: specified Td (%d) is smaller than persistency of excitation condition (%d)\n\n', Td, min_num_data_pts);
end

% Check if Tini satisfies lag condition
Tini_min = compute_observability_index(param.Cdist, param.Adist);
if Tini < Tini_min
    fprintf('WARNING: specified Tini (%d) is smaller than observability index (%d)\n\n', Tini, Tini_min);
end

% Generate sufficiently persistently exciting PRBS signal
min_PRBS_period = param.m * (Tini + exc_param.Nd + 1 + param.n + param.d);
min_PRBS_order = ceil(log(min_PRBS_period + 1) / log(2));
Td_PRBS_order = floor(log(Td + 1) / log(2));
exc_param.PRBS_period = max([2 ^ min_PRBS_order - 1, 2 ^ Td_PRBS_order - 1]);
% exc_param.PRBS_period = 2 ^ min_PRBS_order - 1;
PRBS_num_periods = ceil(Td / exc_param.PRBS_period);
PRBS_band = [0, 1];
PRBS_range = [-1, 1];
PRBS_signal_one_period = idinput([exc_param.PRBS_period, 1, 1], 'prbs', PRBS_band, PRBS_range);
PRBS_signal = repmat(PRBS_signal_one_period, PRBS_num_periods, 1);

PRBS_signal_one_period2 = idinput([exc_param.PRBS_period, 1, PRBS_num_periods], 'prbs', PRBS_band, PRBS_range);

exc_param.T = max([exc_param.PRBS_period Td]);
PRBS_signal = PRBS_signal(1:exc_param.T);


% Excitation simulation time
exc_param.exc_start_time_d = floor(exc_start_time / exc_param.exc_sample_time);
exc_param.exc_signal_time = exc_param.exc_sample_time * exc_param.T * exc_param.exc_num_averages;
exc_param.sim_time = exc_param.exc_start_time_d * exc_param.exc_sample_time + exc_param.exc_signal_time;

exc_param.sim_time = exc_param.sim_time - exc_param.exc_sample_time;
sim_time_vector = (0 : exc_param.exc_sample_time : exc_param.sim_time).';

% Thrust excitation signal
%thrust_exc_amp = 1;
% thrust_exc_amp = 10e-3 * param.g*param.nrotor_vehicle_mass_true;
thrust_exc_amp = 10e-2* param.g;

% amp_aux = [10e-4; 10e-3; 10e-2; 10e-1; 1];
% thrust_exc_amp = amp_aux(i_aux)*param.g;

thrust_PRBS_signal = thrust_exc_amp * PRBS_signal;
exc_param.thrust_exc_signal = repmat(thrust_PRBS_signal, exc_param.exc_num_averages, 1);

% rolld excitation signal
% rolld_exc_amp = 1.8*10e-2;
% rolld_exc_amp = 1*10e-2;

amp_aux = [10e-4; 10e-3; 10e-2; 2*10e-2; 5*10e-2];
rolld_exc_amp = amp_aux(i_aux);

rolld_PRBS_shift_d = floor(exc_param.PRBS_period / param.m * 1);
rolld_PRBS_signal = rolld_exc_amp * circshift(PRBS_signal, rolld_PRBS_shift_d);
exc_param.rolld_exc_signal = repmat(rolld_PRBS_signal, exc_param.exc_num_averages, 1);


% pitchd excitation signal
% pitchd_exc_amp = 1.8*10e-2;
% pitchd_exc_amp = 1*10e-2;

amp_aux = [10e-4; 10e-3; 10e-2; 2*10e-2; 5*10e-2];
pitchd_exc_amp = amp_aux(i_aux);


pitchd_PRBS_shift_d = floor(exc_param.PRBS_period / param.m * 2);
pitchd_PRBS_signal = pitchd_exc_amp * circshift(PRBS_signal, pitchd_PRBS_shift_d);
exc_param.pitchd_exc_signal = repmat(pitchd_PRBS_signal, exc_param.exc_num_averages, 1);
    

% % wZ excitation signal - only excite if DeePC yaw control is enabled
% if param.DeePC_yaw_control
%     %wZ_exc_amp = 1;
%     wZ_exc_amp = 60 * param.deg2rad;
% 
%     wZ_PRBS_shift_d = floor(exc_param.PRBS_period / param.m * 3);
%     wZ_PRBS_signal = wZ_exc_amp * circshift(PRBS_signal, wZ_PRBS_shift_d);
%     exc_param.wZ_exc_signal = repmat(wZ_PRBS_signal, exc_param.exc_num_averages, 1);
% 
% else
%     exc_param.wZ_exc_signal = zeros(length(sim_time_vector), 1);
% end

% % yawd excitation signal
% % yawd_exc_amp = 10e-3;
% yawd_exc_amp = 60 * param.deg2rad*10e-3;
% 
% 
% yawd_PRBS_shift_d = floor(exc_param.PRBS_period / param.m * 3);
% yawd_PRBS_signal = yawd_exc_amp * circshift(PRBS_signal, yawd_PRBS_shift_d);
% exc_param.yawd_exc_signal = repmat(yawd_PRBS_signal, exc_param.exc_num_averages, 1);

exc_param.yawd_exc_signal = zeros(length(sim_time_vector), 1);
    


thrust_exc_ext_signal = [zeros(exc_param.exc_start_time_d, 1);...
    exc_param.thrust_exc_signal];

exc_param.thrust_exc_signal_matrix = [sim_time_vector thrust_exc_ext_signal];


rolld_exc_ext_signal = [zeros(exc_param.exc_start_time_d, 1);...
    exc_param.rolld_exc_signal];

pitchd_exc_ext_signal = [zeros(exc_param.exc_start_time_d, 1);...
    exc_param.pitchd_exc_signal];

% yawd_exc_ext_signal = [zeros(exc_param.exc_start_time_d, 1);...
%     exc_param.yawd_exc_signal];

yawd_exc_ext_signal = exc_param.yawd_exc_signal;

% if param.DeePC_yaw_control
%     wZ_exc_ext_signal = [zeros(exc_param.exc_start_time_d, 1);...
%     exc_param.wZ_exc_signal];
% else
%     wZ_exc_ext_signal = exc_param.wZ_exc_signal;
% end

exc_param.eulerd_exc_signal_matrix = [sim_time_vector rolld_exc_ext_signal pitchd_exc_ext_signal yawd_exc_ext_signal];

fprintf('Done generating excitation signal\n\n');
end