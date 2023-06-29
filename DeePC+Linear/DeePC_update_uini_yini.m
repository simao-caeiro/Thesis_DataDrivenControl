function DeePC_update_uini_yini(input)

%% This function is used in Simulink as interpreted Matlab function

%% GLOBAL VARIABLES
global start_flag uini yini

%% PERSISTENT VARIABLES
% Parameters: Get from workspace
persistent param

if isempty(param) || start_flag
    param = evalin('base', 'param');
end

%% INPUT VARIABLES: [u; y]
% u: Current input [R^n]
% y: Current output [R^p]
start_i = 0;
u = input(start_i+1:start_i+param.m);
start_i = start_i + param.m;
y = input(start_i+1:end);

%% Update uini & yini
uini = circshift(uini, -param.m);
uini(end-param.m+1:end) = u;
yini = circshift(yini, -param.p);
yini(end-param.p+1:end) = y;

end