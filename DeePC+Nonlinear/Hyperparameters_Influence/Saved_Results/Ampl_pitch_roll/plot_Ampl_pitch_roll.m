clc
clear all
close all

%% Variáveis:

n_mat = 5;
amp_aux = [10e-4; 10e-3; 10e-2; 2*10e-2; 5*10e-2]; % order

S_x = zeros(n_mat,1);
S_y = zeros(n_mat,1);
S_z = zeros(n_mat,1);
S_avg = zeros(n_mat,1);

ts_x = zeros(n_mat,1);
ts_y = zeros(n_mat,1);
ts_z = zeros(n_mat,1);
ts_avg = zeros(n_mat,1);
ts_max = zeros(n_mat,1);

e_x = zeros(n_mat,1);
e_y = zeros(n_mat,1);
e_z = zeros(n_mat,1);
e_avg = zeros(n_mat,1);

t_c = zeros(n_mat,1);

aux = zeros(n_mat,1);

for i_aux=1:n_mat

    load(i_aux+".mat");

    S_x(i_aux) = S(1);
    S_y(i_aux) = S(2);
    S_z(i_aux) = S(3);
    S_avg(i_aux) = S(4);

    ts_x(i_aux) = ts(1);
    ts_y(i_aux) = ts(2);
    ts_z(i_aux) = ts(3);
    ts_avg(i_aux) = ts(4);
    ts_max(i_aux) = max(ts);

    e_x(i_aux) = error_track(1);
    e_y(i_aux) = error_track(2);
    e_z(i_aux) = error_track(3);
    e_avg(i_aux) = error_track(4);

    t_c(i_aux) = t_comp*10^3;

    aux(i_aux) = i_aux;

end


%% Remove unnecessary variables
clearvars -except S_x S_y S_z S_avg ts_x ts_y ts_z ts_avg e_x e_y e_z e_avg ...
    t_c aux ts_max amp_aux
 
%% Plots

figure()
plot(aux,S_x,'-o')
hold on
plot(aux,S_y,'-o')
hold on
plot(aux,S_z,'-o')
hold on
plot(aux,S_avg,'-o')
ylabel('S [%]') 
xlabel('Order Amplitude pitch&roll PRBS')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,ts_x,'-o')
hold on
plot(aux,ts_y,'-o')
hold on
plot(aux,ts_z,'-o')
hold on
plot(aux,ts_avg,'-o')
ylabel('Settling Time [s]') 
xlabel('Order Amplitude pitch&roll PRBS')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,e_x,'-o')
hold on
plot(aux,e_y,'-o')
hold on
plot(aux,e_z,'-o')
hold on
plot(aux,e_avg,'-o')
ylabel('Tracking Error [m]') 
xlabel('Order Amplitude pitch&roll PRBS')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,t_c,'-o')
ylabel('Computational Time [s]') 
xlabel('Order Amplitude pitch&roll PRBS')
grid on

figure()
plot(aux,S_avg,'-o')
ylabel('Average S [%]') 
xlabel('Order Amplitude pitch&roll PRBS')
grid on

figure()
plot(aux,ts_max,'-o')
ylabel('Max Settling Time [s]') 
xlabel('Order Amplitude pitch&roll PRBS')
grid on

figure()
plot(aux,e_avg,'-o')
ylabel('Average Tracking Error [m]') 
xlabel('Order Amplitude pitch&roll PRBS')
grid on


