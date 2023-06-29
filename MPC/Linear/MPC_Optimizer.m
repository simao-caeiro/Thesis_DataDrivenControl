function output = MPC_Optimizer(full_state_meas)

%% This function is used in Simulink as interpreted Matlab function

%% GLOBAL VARIABLES
global start_flag

%% PERSISTENT VARIABLES
% Parameters: Get from workspace
persistent param mpc_param U_mpc Xd_mpc k_aux_mpc

if isempty(param) || start_flag
    param = evalin('base', 'param');
    mpc_param = evalin('base', 'mpc_param');
    U_mpc = evalin('base', 'U_mpc');
    Xd_mpc = evalin('base', 'Xd_mpc');
    k_aux_mpc = evalin('base', 'k_aux_mpc');
    
    disp('Initialization Successful');
    start_flag = false;
end


Xd_mpc(:,k_aux_mpc) = full_state_meas;

Dxdk = Xd_mpc(:,k_aux_mpc)-Xd_mpc(:,k_aux_mpc-1);
xk = [Dxdk; mpc_param.Cd*Xd_mpc(:,k_aux_mpc)];

% compute unconstrained optimal sequence and MPC policy
dUopt(:,:,k_aux_mpc) = reshape(-(mpc_param.K*xk - mpc_param.Ky*mpc_param.Yb) ,mpc_param.nu,mpc_param.N);
Uopt(:,:,k_aux_mpc) = U_mpc(:,k_aux_mpc-1) + dUopt(:,:,k_aux_mpc);
U_mpc(:,k_aux_mpc) = Uopt(:,1,k_aux_mpc);

U_output = U_mpc(:,k_aux_mpc);

k_aux_mpc = k_aux_mpc+1;

output = U_output;

end