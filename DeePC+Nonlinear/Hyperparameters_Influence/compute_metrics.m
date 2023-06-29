function [ts, S, t_comp, error_track] = ...
    compute_metrics(signal_x, signal_y, signal_z, comp_time, ref, Ts, start_time)

ts = zeros(4,1);
t_comp = 0;
S = zeros(4,1);
error_track = zeros(4,1);

ref_x = ref(1);
ref_y = ref(2);
ref_z = ref(3);


%% Overshoot calculation
% X
S_x = compute_overshoot(signal_x, ref_x);

% Y
S_y = compute_overshoot(signal_y, ref_y);

% Z
S_z = compute_overshoot(signal_z, ref_z);

% average value
S_avg = (S_x+S_y+S_z)/3;

S = [S_x; S_y; S_z; S_avg];

%% Settling time calculation
percent = 2;

% X
ts_x = compute_settling_time(signal_x, ref_x, percent, Ts);
ts_x = ts_x - start_time;

% Y
ts_y = compute_settling_time(signal_y, ref_y, percent, Ts);
ts_y = ts_y - start_time;

% Z
ts_z = compute_settling_time(signal_z, ref_z, percent, Ts);
ts_z = ts_z - start_time;

% average value
ts_avg = (ts_x+ts_y+ts_z)/3;

ts = [ts_x; ts_y; ts_z; ts_avg];

%% Tracking error calculation
n_samples_x = ceil((ts_x/Ts)*1/3);
n_samples_y = ceil((ts_x/Ts)*1/3);
n_samples_z = ceil((ts_x/Ts)*1/3);

% X
e_x = compute_tracking_error(signal_x, ref_x, n_samples_x);

% Y
e_y = compute_tracking_error(signal_y, ref_y, n_samples_y);

% Z
e_z = compute_tracking_error(signal_z, ref_z, n_samples_z);

% average value
e_avg = (e_x+e_y+e_z)/3;

error_track = [e_x; e_y; e_z; e_avg];

%% Average Algorithm Compution Time calculation

t_comp = mean(comp_time);

end




