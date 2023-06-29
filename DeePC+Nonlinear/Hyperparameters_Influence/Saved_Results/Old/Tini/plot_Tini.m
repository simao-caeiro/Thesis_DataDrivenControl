clc
clear all
close all

%% Variáveis:

S_x = zeros(10,1);
S_y = zeros(10,1);
S_z = zeros(10,1);
S_avg = zeros(10,1);

ts_x = zeros(10,1);
ts_y = zeros(10,1);
ts_z = zeros(10,1);
ts_avg = zeros(10,1);
ts_max = zeros(10,1);

e_x = zeros(10,1);
e_y = zeros(10,1);
e_z = zeros(10,1);
e_avg = zeros(10,1);

t_c = zeros(10,1);

aux = zeros(10,1);

for i_aux=1:10

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

    t_c(i_aux) = t_comp;

    aux(i_aux) = i_aux+2;

end


%% Remove unnecessary variables
clearvars -except S_x S_y S_z S_avg ts_x ts_y ts_z ts_avg e_x e_y e_z e_avg ...
    t_c aux ts_max
 
%% Plots

figure()
plot(aux,S_x)
hold on
plot(aux,S_y)
hold on
plot(aux,S_z)
hold on
plot(aux,S_avg)
ylabel('S [%]') 
xlabel('Tini')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,ts_x)
hold on
plot(aux,ts_y)
hold on
plot(aux,ts_z)
hold on
plot(aux,ts_avg)
ylabel('Settling Time [s]') 
xlabel('Tini')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,e_x)
hold on
plot(aux,e_y)
hold on
plot(aux,e_z)
hold on
plot(aux,e_avg)
ylabel('Tracking Error [m]') 
xlabel('Tini')
legend('X','Y','Z','Avg','Location','best')
grid on

figure()
plot(aux,t_c)
ylabel('Computational Time [s]') 
xlabel('Tini')
grid on

figure()
plot(aux,S_avg)
ylabel('Average S [%]') 
xlabel('Tini')
grid on

figure()
plot(aux,ts_max)
ylabel('Max Settling Time [s]') 
xlabel('Tini')
grid on

figure()
plot(aux,e_avg)
ylabel('Average Tracking Error [m]') 
xlabel('Tini')
grid on


