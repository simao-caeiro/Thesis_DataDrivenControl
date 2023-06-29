function [u_data_exc, y_data_exc, u_data_init, y_data_init, yaw_data] =...
    collect_DeePC_data(param, exc_param, Tini)
% Data collection mode input:
% 0 = linear model with gravity
% 1 = non-linear model

% Initialize uini yini
global start_flag uini yini
start_flag = true;
uini = zeros(param.m * Tini, 1);
yini = zeros(param.p * Tini, 1);

% Switch to data collection noise seeds
param.measurement_noise_full_state_seed = param.measurement_noise_full_state_seed_data;
param.measurement_noise_gyroscope_seed = param.measurement_noise_gyroscope_seed_data;
param.measurement_noise_accelerometer_seed = param.measurement_noise_accelerometer_seed_data;
assignin('base','param', param);

sim('Iris_PID_Gazebo_Excitation');
% sim('Iris_PID_Gazebo_Excitation_aux');


% Switch back to running noise seeds
param.measurement_noise_full_state_seed = param.measurement_noise_full_state_seed_run;
param.measurement_noise_gyroscope_seed = param.measurement_noise_gyroscope_seed_run;
param.measurement_noise_accelerometer_seed = param.measurement_noise_accelerometer_seed_run;
assignin('base','param', param);

eulerd_cmd = reshape(eulerd_cmd,size(eulerd_cmd,1),size(eulerd_cmd,3))';

u_data_raw = [total_thrust_cmd eulerd_cmd].';
u_data_raw = eye(param.m, 4) * u_data_raw;
% yaw_rate_cmd_data = body_rates_cmd(:,end).';

x_data_raw = full_state_meas(:,1:9).';
yaw_data = x_data_raw(end,:);

u_data_init = u_data_raw(:,1:exc_param.exc_start_time_d);
x_data_init = x_data_raw(:,1:exc_param.exc_start_time_d);
y_data_init = param.Cd * x_data_init;

u_data_exc_raw = u_data_raw(:,exc_param.exc_start_time_d+1:end);
x_data_exc_raw = x_data_raw(:,exc_param.exc_start_time_d+1:end);
y_data_exc_raw = param.Cd * x_data_exc_raw;


u_data_exc_gen = u_data_exc_raw;
y_data_exc_gen = y_data_exc_raw;


u_data_exc = zeros(param.m, exc_param.T);
y_data_exc = zeros(param.p, exc_param.T);
% Skip first period if averaging to neglect transient effect
if exc_param.exc_num_averages == 1
    start_i = 1;
else
    start_i = 2;
end
for i = start_i : exc_param.exc_num_averages
    u_data_exc = u_data_exc + u_data_exc_gen(:,(i-1)*exc_param.T+1:i*exc_param.T) / exc_param.exc_num_averages;
    y_data_exc = y_data_exc + y_data_exc_gen(:,(i-1)*exc_param.T+1:i*exc_param.T) / exc_param.exc_num_averages;
end

H_u_pe = data2hankel(u_data_exc, Tini + exc_param.Nd + 1 + param.n + param.d);
rank(H_u_pe, 1e-3);
if rank(H_u_pe, 1e-3) == size(H_u_pe,1)
    fprintf('Data persistenly exciting of order Tini + Nd + 1 + n + d\n');
else
    fprintf('Data *NOT* persistenly exciting\n');
end

% total_thrust_collected = total_thrust_cmd(126:end);
% total_thrust_LQR = total_thrust_LQR(126:end);
% total_thrust_PRBS = total_thrust_PRBS(126:end);
% 
% eulerd_collected = eulerd_cmd(126:end,:);
% eulerd_LQR = reshape(eulerd_LQR,size(eulerd_LQR,1),size(eulerd_LQR,3)).';
% eulerd_LQR = eulerd_LQR(126:end,:);
% eulerd_PRBS = eulerd_PRBS(126:end,:);
% 
% assignin('base','total_thrust_collected', total_thrust_collected);
% assignin('base','total_thrust_LQR', total_thrust_LQR);
% assignin('base','total_thrust_PRBS', total_thrust_PRBS);
% assignin('base','eulerd_collected', eulerd_collected);
% assignin('base','eulerd_LQR', eulerd_LQR);
% assignin('base','eulerd_PRBS', eulerd_PRBS);


end