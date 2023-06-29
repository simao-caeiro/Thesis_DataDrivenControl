function fg = plot_DeePC(param, exc_param, DeePC_param, DeePC_param_perf,...
    total_thrust_cmd, body_rates_cmd, full_state_act, full_state_meas,...
    r_out, g_out, sigma_out, solve_time_out,...
    g_perf_out, solve_time_perf_out,...
    Tini)

close all;

%% Screen size used to place plots
screensize = get(groot, 'ScreenSize');
screenwidth = screensize(3) - screensize(3)/4;
screenheight = screensize(4) - screensize(4)/4;
default_width = screenwidth / 2;
default_height = screenheight / 2;
fg = 1;

%% Some text for labels
u_text = ["$F$ $[N]$"; "$\omega^{(B)}_{x}$ $[\frac{rad}{s}]$";...
    "$\omega^{(B)}_{y}$ $[\frac{rad}{s}]$"; "$\omega^{(B)}_{z}$ $[\frac{rad}{s}]$"];
upred_text = ["$F_{pred}$ $[N]$"; "$\omega^{(B)}_{x_{pred}}$ $[\frac{rad}{s}]$";...
    "$\omega^{(B)}_{y_{pred}}$ $[\frac{rad}{s}]$"; "$\omega^{(B)}_{z_{pred}}$ $[\frac{rad}{s}]$"];
upred_perf_text = ["$F_{pred_{perf}}$ $[N]$"; "$\omega^{(B)}_{x_{pred_{perf}}}$ $[\frac{rad}{s}]$";...
    "$\omega^{(B)}_{y_{pred_{perf}}}$ $[\frac{rad}{s}]$"; "$\omega^{(B)}_{z_{pred_{perf}}}$ $[\frac{rad}{s}]$"];
x_text = ["$p^{(I)}_x$ $[m]$"; "$p^{(I)}_y$ $[m]$"; "$p^{(I)}_z$ $[m]$";...
    "$\dot{p}^{(I)}_{x}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{y}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{z}$ $[\frac{m}{s}]$";...
    "$\gamma$ $[rad]$"; "$\beta$ $[rad]$"; "$\alpha$ $[rad]$"];
i = find(param.Cd(1,:) == 1);
y_text = x_text(i);
for i = 2 : size(param.Cd, 1)
    i = find(param.Cd(i,:) == 1);
    y_text = [y_text; x_text(i)];
end
yref_text = y_text(DeePC_param.ref_outputs);
x_r_text = ["$p^{(I)}_{x_{ref}}$ $[m]$"; "$p^{(I)}_{y_{ref}}$ $[m]$"; "$p^{(I)}_{z_{ref}}$ $[m]$";...
    "$\dot{p}^{(I)}_{x_{ref}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{y_{ref}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{z_{ref}}$ $[\frac{m}{s}]$";...
    "$\gamma_{ref}$ $[rad]$"; "$\beta_{ref}$ $[rad]$"; "$\alpha_{ref}$ $[rad]$"];
r_text = x_r_text(DeePC_param.ref_states(1));
for i = 2 : length(DeePC_param.ref_states)
    r_text = [r_text; x_r_text(DeePC_param.ref_states(i))];
end
xpred_text = ["$p^{(I)}_{x_{pred}}$ $[m]$"; "$p^{(I)}_{y_{pred}}$ $[m]$"; "$p^{(I)}_{z_{pred}}$ $[m]$";...
    "$\dot{p}^{(I)}_{x_{pred}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{y_{pred}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{pred}}$ $[\frac{m}{s}]$";...
    "$\gamma_{pred}$ $[rad]$"; "$\beta_{pred}$ $[rad]$"; "$\alpha_{pred}$ $[rad]$"];
xpred_perf_text = ["$p^{(I)}_{x_{pred_{perf}}}$ $[m]$"; "$p^{(I)}_{y_{pred_{perf}}}$ $[m]$"; "$p^{(I)}_{z_{pred_{perf}}}$ $[m]$";...
    "$\dot{p}^{(I)}_{x_{pred_{perf}}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{y_{pred_{perf}}}$ $[\frac{m}{s}]$"; "$\dot{p}^{(I)}_{z_{pred_{perf}}}$ $[\frac{m}{s}]$";...
    "$\gamma_{pred_{perf}}$ $[rad]$"; "$\beta_{pred_{perf}}$ $[rad]$"; "$\alpha_{pred_{perf}}$ $[rad]$"];
ypred_text = xpred_text(DeePC_param.ref_states(1));
ypred_perf_text = xpred_perf_text(DeePC_param.ref_states(1));
for i = 2 : length(DeePC_param.ref_states)
    ypred_text = [ypred_text; xpred_text(DeePC_param.ref_states(i))];
    ypred_perf_text = [ypred_perf_text; xpred_perf_text(DeePC_param.ref_states(i))];
end

%% Organize data for plotting
u = [total_thrust_cmd body_rates_cmd].';
u = eye(param.m, 4) * u;
x = full_state_act(:,1:9).';
x_meas = full_state_meas(:,1:9).';
y = param.Cd * x;
y_meas = param.Cd * x_meas;
yref = y(DeePC_param.ref_outputs,:);
yref_meas = y_meas(DeePC_param.ref_outputs,:);
r = r_out.';
T_eq = param.nrotor_vehicle_mass_true*param.g;
u_ref = [T_eq;zeros(param.m-1,1)];

%% Find cost
% output_cost = diag((yref - r).' * (yref - r)).';
% input_cost = diag(u.' * u).';

output_cost = diag((y - r).' * (y - r)).';
input_cost = diag((u - u_ref).' * (u - u_ref)).';

%% Find norms of g & sigma and fill with zeros prior to activation of DeePC
sim_length = size(u, 2);
DeePC_length = size(g_out, 1);
g_norm = vecnorm(g_out.');
g_norm = [zeros(1, sim_length - DeePC_length) g_norm];
sigma_norm = vecnorm(sigma_out.');
sigma_norm = [zeros(1, sim_length - DeePC_length) sigma_norm];

%% Find yini_act - (yini_meas + sigma)
yini_error = zeros(1, DeePC_length);
DeePC_start_i = sim_length - DeePC_length;
for i = 1 : DeePC_length
    start_i = DeePC_start_i + i - Tini;
    end_i = DeePC_start_i + i - 1;
    yini_act = reshape(y(:,start_i:end_i), [], 1);
    yini_DeePC = DeePC_param.Y_p * g_out(i,:).';
    yini_error(i) = norm(yini_act - yini_DeePC);
end
yini_error = [zeros(1, sim_length - DeePC_length) yini_error];

%% Overview plot
figure(fg);
fg = fg + 1;
fig = gcf;
fig.Position = [0, 0, screenwidth, screenheight];

% Reference vs. output plot
subplot(3,2,1);
plot(r(1,:));
lgd_text = r_text(1);
hold on;
plot(yref(1,:));
lgd_text = [lgd_text, yref_text(1)];
for i = 2 : length(DeePC_param.ref_outputs)
    plot(r(i,:));
    lgd_text = [lgd_text, r_text(i)];
    plot(yref(i,:));
    lgd_text = [lgd_text, yref_text(i)];
end
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Reference vs. Output';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'southeast';

% Costs plot
subplot(3,2,2);
plot(output_cost);
lgd_text = "Output Cost";
hold on;
plot(input_cost);
lgd_text = [lgd_text, "Input Cost"];
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Cost';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'northeast';

% Total thrust command plot
subplot(3,2,3);
plot(u(1,:));
lgd_text = u_text(1);
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Total Thrust Command';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'northeast';

% Body rates command plot
subplot(3,2,4);
plot(u(2,:));
lgd_text = u_text(2);
hold on;
for i = 3 : param.m
    plot(u(i,:));
    lgd_text = [lgd_text, u_text(i)];
end
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Body Rates Command';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'northeast';

% Norm of g plot
subplot(3,2,5);
plot(g_norm);
lgd_text = "$\left\|g\right\|_{2}$";
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = '2-norm of g';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'southeast';

% Norm of sigma & yini_act - (yini_meas + sigma) plot
subplot(3,2,6);
plot(sigma_norm);
lgd_text = "$\left\|\sigma\right\|_{2}$";
hold on;
plot(yini_error);
lgd_text = [lgd_text, "$\left\|yini_{act}-yini_{DeePC}\right\|_{2}$"];
axis tight;
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = '2-norm of sigma vs. 2-norm of yini error';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'southeast';

%% Solve time plot
figure(fg);
fg = fg + 1;
fig = gcf;
fig.Position = [0, 0, screenwidth, screenheight];
plot(solve_time_out);
lgd_text = "$t_{solve}$";
if param.compare_to_perf
    hold on;
    plot(solve_time_perf_out);
    lgd_text = [lgd_text, "$t_{solve_{perf}}$"];
end
axis tight;
max_solve_time = max([max(solve_time_out) max(solve_time_perf_out)]);
if max_solve_time > 0
    ylim([0 max_solve_time]);
end
axes = gca;
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Optimization Solve Time';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
axes.YLabel.Interpreter = 'latex';
axes.YLabel.String = '$t_{solve}$ $[sec]$';
axes.YLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 12;
lgd.Location = 'northeast';

%% Predictions plot
figure(fg);
fg = fg + 1;
fig = gcf;
fig.Position = [0, 0, default_width, default_height];

% Reference vs. measured output plot
% If reference is identical across all outputs, only plot once as 'ref'
same_ref = true;
for i = 1 : length(DeePC_param.ref_outputs)
    for j = i + 1 : length(DeePC_param.ref_outputs)
        if ~isequal(r(i,:), r(j,:))
            same_ref = false;
            break;
        end
        if ~same_ref
            break;
        end
    end
end
subplot(3,2,1:4);
plot(r(1,:), '--', 'LineWidth', 2);
if same_ref
    lgd_text = "$ref$";
else
    lgd_text = r_text(1);
end
hold on;
plot(yref_meas(1,:), 'LineWidth', 2);
lgd_text = [lgd_text, yref_text(1)];
for i = 2 : length(DeePC_param.ref_outputs)
    if ~same_ref
        plot(r(i,:), '--', 'LineWidth', 2);
        lgd_text = [lgd_text, r_text(i)];
    end
    plot(yref_meas(i,:), 'LineWidth', 2);
    lgd_text = [lgd_text, yref_text(i)];
end
if param.plot_predictions
    % y Predictions
    y_pred = DeePC_param.Y_f * g_out(1,:).';
    y_pred = reshape(y_pred, param.p, []);
    y_pred_ref = y_pred(DeePC_param.ref_outputs,:);
    y_pred_ind = DeePC_start_i + 1 : DeePC_start_i + 1 + exc_param.Nd;
    ypred_plot(1) = plot(y_pred_ind, y_pred_ref(1,:), ':', 'LineWidth', 2);
    lgd_text = [lgd_text, ypred_text(1)];
    ypred_plot(1).XDataSource = 'y_pred_ind';
    ypred_plot(1).YDataSource = 'y_pred_ref(1,:)';
    for i = 2 : length(DeePC_param.ref_outputs)
        ypred_plot(i) = plot(y_pred_ind, y_pred_ref(i,:), ':', 'LineWidth', 2);
        lgd_text = [lgd_text, ypred_text(i)];
        ypred_plot(i).XDataSource = 'y_pred_ind';
        ypred_plot(i).YDataSource = strcat('y_pred_ref(', num2str(i), ',:)');
    end
end
axes = gca;
y_semi_tight(axes, 1.2);
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Reference vs. Output';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 10;
lgd.Location = 'southeast';
%lgd.Location = 'northwest';
%lgd.Location = 'bestoutside';

% Total thrust command plot
subplot(3,2,5);
plot(u(1,:), 'LineWidth', 2);
lgd_text = u_text(1);
if param.plot_predictions
    hold on;
    % Thrust Predictions
    u_pred = DeePC_param.U_f * g_out(1,:).';
    u_pred = reshape(u_pred, param.m, []);
    u_pred_ind = y_pred_ind(1:end-1);
    upred_plot(1) = plot(u_pred_ind, u_pred(1,:), ':', 'LineWidth', 2);
    lgd_text = [lgd_text, upred_text(1)];
    upred_plot(1).XDataSource = 'u_pred_ind';
    upred_plot(1).YDataSource = 'u_pred(1,:)';   
end
axes = gca;
y_semi_tight(axes, 1.2);
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Total Thrust Command';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 10;
lgd.Location = 'northeast';
%lgd.Location = 'southwest';
%lgd.Location = 'bestoutside';

% Body rates command plot
subplot(3,2,6);
plot(u(2,:), 'LineWidth', 2);
lgd_text = u_text(2);
hold on;
for i = 3 : param.m
    plot(u(i,:), 'LineWidth', 2);
    lgd_text = [lgd_text, u_text(i)];
end
if param.plot_predictions
    % Body rates predictions
    for i = 2 : param.m
        upred_plot(i) = plot(u_pred_ind, u_pred(i,:), ':', 'LineWidth', 2);
        lgd_text = [lgd_text, upred_text(i)];
        upred_plot(i).XDataSource = 'u_pred_ind';
        upred_plot(i).YDataSource = strcat('u_pred(', num2str(i), ',:)');
    end
end
axes = gca;
y_semi_tight(axes, 1.2);
axes.Title.Interpreter = 'latex';
axes.Title.String = 'Body Rates Command';
axes.Title.FontSize = 18;
axes.XAxis.TickLabelInterpreter = 'latex';
axes.XAxis.FontSize = 10;
axes.YAxis.TickLabelInterpreter = 'latex';
axes.YAxis.FontSize = 10;
axes.XLabel.Interpreter = 'latex';
axes.XLabel.String = '$t$ $[timesteps]$';
axes.XLabel.FontSize = 14;
lgd = legend(lgd_text);
lgd.Interpreter = 'latex';
lgd.FontSize = 10;
lgd.Location = 'northeast';
%lgd.Location = 'southwest';
%lgd.Location = 'bestoutside';

if param.plot_predictions
% Update predictions on button click
w = waitforbuttonpress;
    if w
        for i = 2 : DeePC_length
            % y
            y_pred = DeePC_param.Y_f * g_out(i,:).';
            y_pred = reshape(y_pred, param.p, []);
            y_pred_ref = y_pred(DeePC_param.ref_outputs,:);
            y_pred_ind = DeePC_start_i + i : DeePC_start_i + i + exc_param.Nd;
            refreshdata(ypred_plot, 'caller');
            % 'Perfect' y
            if param.compare_to_perf
                y_pred_perf = DeePC_param_perf.Y_f * g_perf_out(i,:).';
                y_pred_perf = reshape(y_pred_perf, param.p, []);
                y_pred_perf_ref = y_pred_perf(DeePC_param.ref_outputs,:);
                refreshdata(ypred_perf_plot, 'caller');
            end

            % u
            u_pred = DeePC_param.U_f * g_out(i,:).';
            u_pred = reshape(u_pred, param.m, []);
            u_pred_ind = y_pred_ind(1:end-1);
            refreshdata(upred_plot, 'caller');
            % 'Perfect' u
            if param.compare_to_perf
                u_pred_perf = DeePC_param_perf.U_f * g_perf_out(i,:).';
                u_pred_perf = reshape(u_pred_perf, param.m, []);
                refreshdata(upred_perf_plot, 'caller');
            end

            w = waitforbuttonpress;
            if ~w
                break;
            end
        end
    end
end

fprintf('Done plotting\n');
end

function y_semi_tight(ax, scale)
    axis tight; % Set axis tight
    yl = ylim(ax); % Get tight axis limits
    range = yl(2) - yl(1); % Get tight axis range
    sc_range = scale * range; % Scale range
    yl(1) = yl(1) - (sc_range - range) / 2; % New ymin
    yl(2) = yl(1) + sc_range; % New ymax
    ylim(ax, yl);
end