clear;
close all;
clc;

%%
min_PRBS_order = 6;
PRBS_period = 2 ^ min_PRBS_order - 1;
PRBS_num_periods = 1;
PRBS_band = [0, 1/2];
PRBS_range = [-1, 1];
PRBS_signal1 = idinput([PRBS_period, 1, PRBS_num_periods], 'prbs', PRBS_band, PRBS_range);


sample_time = 0.04;
sim_time = sample_time*length(PRBS_signal1);
sim_time_vector = (0 : sample_time : sim_time-sample_time).';

%%
myFigure1 = figure();
plot(sim_time_vector,PRBS_signal1, 'LineWidth',1.5)
ylabel('PRBS Amplitude') 
xlabel('Time [s]')
grid on
ylim([-1.2 1.2])
xlim([0 inf])



%%
min_PRBS_order = 6;
PRBS_period = 2 ^ min_PRBS_order - 1;
PRBS_num_periods = 1;
PRBS_band = [0, 1];
PRBS_range = [-1, 1];
PRBS_signal2 = idinput([PRBS_period, 1, PRBS_num_periods], 'prbs', PRBS_band, PRBS_range);


sample_time = 0.04;
sim_time = sample_time*length(PRBS_signal2);
sim_time_vector = (0 : sample_time : sim_time-sample_time).';

%%
myFigure2 = figure();
plot(sim_time_vector,PRBS_signal2, 'LineWidth',1.5)
ylabel('PRBS Amplitude') 
xlabel('Time [s]')
grid on
ylim([-1.2 1.2])
xlim([0 inf])


% saveFigAsPDF(myFigure1,'Figures_PDF/PRBS_v1');
% saveFigAsPDF(myFigure2,'Figures_PDF/PRBS_v2');

%% 
% myFigure1 = figure();
% plot(sim_time_vector,PRBS_signal1, 'LineWidth',1.5)
% hold on
% plot(sim_time_vector,PRBS_signal2, 'LineWidth',1.5)
% ylabel('PRBS Amplitude') 
% xlabel('Time [s]')
% grid on
% ylim([-1.2 1.2])
% xlim([0 inf])