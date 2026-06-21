%% lqr_sensitivity.m
% LQR R-scaling sensitivity for the nonlinear PVTOL simulation.
%
% Purpose:
%   Sweep the LQR input penalty R and quantify the design trade-off between
%   gain magnitude, control effort, pitch excursion, final position error,
%   and closed-loop pole location.
%
% Outputs:
%   PVTOL_Figures/lqr_R_sensitivity.csv
%   PVTOL_Figures/09_lqr_R_sensitivity.png
%   PVTOL_Figures/09b_lqr_R_tradeoff_summary.png
%
% Run after:
%   design.m

clear; clc; close all;

%% Load parameters and nominal design
SystemParameters

if ~exist('studentID','var')
    studentID = 10695141;
end

assert(exist('designValues.mat','file') == 2, ...
    'designValues.mat not found. Run design.m first.');

S = load('designValues.mat');

Gd = S.Gd;
Hd = S.Hd;
C  = S.C;
Q  = S.Q;
R  = S.R;
L  = S.L;

if isfield(S,'WN')
    WN = S.WN;
else
    WN = 0.626;
end

if isfield(S,'THETA_VALID')
    THETA_VALID = S.THETA_VALID;
else
    THETA_VALID = 0.35;
end

if ~exist('PVTOL_Figures','dir')
    mkdir('PVTOL_Figures');
end

%% Sweep settings
R_scales = [0.25 0.5 1 2 5 10];
t_span   = [0 30];

results = zeros(numel(R_scales), 6);

fprintf('\n=== LQR R-scaling sensitivity ===\n');
fprintf('%8s | %8s | %10s | %12s | %10s | %10s\n', ...
    'R scale', 'max|K|', 'max|u1|', 'max|theta|', 'final err', 'max|z|');
fprintf('%s\n', repmat('-', 1, 78));

%% Run sweep
for i = 1:numel(R_scales)
    rho = R_scales(i);

    % Recompute LQR gain with scaled input penalty.
    K_test = dlqr(Gd, Hd, Q, rho*R);
    e_test = eig(Gd - Hd*K_test);

    % Reset observer log.
    global OBS_LOG
    OBS_LOG = [];

    % Controller handle using this test gain.
    ctrl = @(t,y) sensitivity_controller(t, y, K_test, L, Gd, Hd, C, ...
                                         m, g, Ts, WN);

    % Nonlinear simulation.
    [T, Y, U] = System_Simulator(@VTOL_Dynamics_corrected, x0, t_span, ...
                                 ctrl, Ts, studentID);

    % Reconstruct reference history.
    ref_state = zeros(numel(T), 6);
    for k = 1:numel(T)
        rc = reference_signal(T(k));
        ref_state(k,1) = rc(1);
        ref_state(k,3) = rc(2);
    end

    final_err = norm([Y(end,1)-ref_state(end,1), ...
                      Y(end,3)-ref_state(end,3)]);

    max_theta = max(abs(Y(:,5)));
    max_u1    = max(abs(U(:,1)));
    maxK      = max(abs(K_test(:)));
    maxPole   = max(abs(e_test));

    results(i,:) = [rho, maxK, max_u1, max_theta, final_err, maxPole];

    fprintf('%8.2f | %8.3f | %10.4f | %12.4f | %10.4f | %10.4f\n', ...
        rho, maxK, max_u1, max_theta, final_err, maxPole);
end

%% Save results
T_results = array2table(results, ...
    'VariableNames', {'R_scale','max_abs_K','max_abs_u1_N', ...
                      'max_abs_theta_rad','final_error_m','max_abs_pole'});

writetable(T_results, fullfile('PVTOL_Figures','lqr_R_sensitivity.csv'));

fprintf('\nSaved: PVTOL_Figures/lqr_R_sensitivity.csv\n');

%% Report plot style
LW_MAIN   = 5.0;
LW_AUX    = 3.4;
LW_MARKER = 3.4;

FS_AXES   = 21;
FS_LABEL  = 23;
FS_TITLE  = 25;
FS_LEGEND = 18;
FS_TEXT   = 17;

nominal_rho = 1.0;

%% ========================================================================
% FIGURE 1: Four-panel R-sensitivity plot
% ========================================================================

fig = figure('Color','w', ...
             'Name','LQR R-scaling sensitivity', ...
             'Position',[100 100 2400 1700], ...
             'Renderer','opengl');

tl = tiledlayout(fig, 2, 2, ...
    'Padding','compact', ...
    'TileSpacing','compact');

title(tl, 'LQR Input-Penalty Sensitivity', ...
      'FontWeight','bold', ...
      'FontSize',FS_TITLE + 2);

% ------------------------------
% 1. Gain magnitude
% ------------------------------
ax1 = nexttile(tl);
semilogx(ax1, results(:,1), results(:,2), 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.10 0.35 0.85], ...
    'MarkerEdgeColor','k');

hold(ax1,'on');
xline(ax1, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'LabelVerticalAlignment','bottom', ...
    'LabelOrientation','horizontal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax1,'on');
box(ax1,'on');
xlabel(ax1,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax1,'max |K|', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax1,'Gain magnitude', 'FontWeight','bold', 'FontSize',FS_TITLE);
format_axes(ax1, FS_AXES);

% ------------------------------
% 2. Moment-channel effort
% ------------------------------
ax2 = nexttile(tl);
semilogx(ax2, results(:,1), results(:,3), 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.85 0.25 0.10], ...
    'MarkerEdgeColor','k');

hold(ax2,'on');
xline(ax2, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'LabelVerticalAlignment','bottom', ...
    'LabelOrientation','horizontal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax2,'on');
box(ax2,'on');
xlabel(ax2,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax2,'max |u_1| (N)', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax2,'Moment-channel effort', 'FontWeight','bold', 'FontSize',FS_TITLE);
format_axes(ax2, FS_AXES);

% ------------------------------
% 3. Pitch excursion
% ------------------------------
ax3 = nexttile(tl);
semilogx(ax3, results(:,1), results(:,4), 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.15 0.65 0.25], ...
    'MarkerEdgeColor','k');

hold(ax3,'on');

yline(ax3, THETA_VALID, 'r--', ...
    'LineWidth',LW_AUX, ...
    'Label','validity limit', ...
    'LabelHorizontalAlignment','left', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

xline(ax3, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'LabelVerticalAlignment','bottom', ...
    'LabelOrientation','horizontal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax3,'on');
box(ax3,'on');
xlabel(ax3,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax3,'max |\theta| (rad)', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax3,'Pitch excursion', 'FontWeight','bold', 'FontSize',FS_TITLE);
format_axes(ax3, FS_AXES);

% ------------------------------
% 4. Final tracking error
% ------------------------------
ax4 = nexttile(tl);
semilogx(ax4, results(:,1), results(:,5), 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.45 0.20 0.75], ...
    'MarkerEdgeColor','k');

hold(ax4,'on');
xline(ax4, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'LabelVerticalAlignment','bottom', ...
    'LabelOrientation','horizontal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax4,'on');
box(ax4,'on');
xlabel(ax4,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax4,'final error (m)', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax4,'Residual tracking error', 'FontWeight','bold', 'FontSize',FS_TITLE);
format_axes(ax4, FS_AXES);

exportgraphics(fig, fullfile('PVTOL_Figures','09_lqr_R_sensitivity.png'), ...
    'Resolution',900, ...
    'BackgroundColor','white');

fprintf('Saved: PVTOL_Figures/09_lqr_R_sensitivity.png\n');

%% ========================================================================
% FIGURE 2: Compact two-panel trade-off summary
% Better for two-column report if the four-panel plot is too dense.
% ========================================================================

fig2 = figure('Color','w', ...
              'Name','LQR R Tradeoff Summary', ...
              'Position',[120 120 2400 1100], ...
              'Renderer','opengl');

tl2 = tiledlayout(fig2, 1, 2, ...
    'Padding','compact', ...
    'TileSpacing','compact');

title(tl2, 'LQR R-Scaling Trade-off Summary', ...
      'FontWeight','bold', ...
      'FontSize',FS_TITLE + 2);

% Normalised trade-off plot
ax5 = nexttile(tl2);

normK   = results(:,2) / results(results(:,1)==1,2);
normU1  = results(:,3) / results(results(:,1)==1,3);
normErr = results(:,5) / results(results(:,1)==1,5);

semilogx(ax5, results(:,1), normK, 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.10 0.35 0.85], ...
    'MarkerEdgeColor','k', ...
    'DisplayName','max |K|');

hold(ax5,'on');

semilogx(ax5, results(:,1), normU1, 's-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.85 0.25 0.10], ...
    'MarkerEdgeColor','k', ...
    'DisplayName','max |u_1|');

semilogx(ax5, results(:,1), normErr, '^-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',13, ...
    'MarkerFaceColor',[0.45 0.20 0.75], ...
    'MarkerEdgeColor','k', ...
    'DisplayName','final error');

xline(ax5, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax5,'on');
box(ax5,'on');
xlabel(ax5,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax5,'normalised value', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax5,'Normalised design trade-off', 'FontWeight','bold', 'FontSize',FS_TITLE);
legend(ax5,'Location','best', ...
       'FontWeight','bold', ...
       'FontSize',FS_LEGEND);
format_axes(ax5, FS_AXES);

% Pitch validity plot
ax6 = nexttile(tl2);

semilogx(ax6, results(:,1), results(:,4), 'o-', ...
    'LineWidth',LW_MAIN, ...
    'MarkerSize',14, ...
    'MarkerFaceColor',[0.15 0.65 0.25], ...
    'MarkerEdgeColor','k');

hold(ax6,'on');

yline(ax6, THETA_VALID, 'r--', ...
    'LineWidth',LW_AUX, ...
    'Label','validity limit', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

xline(ax6, nominal_rho, 'k--', ...
    'LineWidth',LW_AUX, ...
    'Label','nominal', ...
    'FontWeight','bold', ...
    'FontSize',FS_TEXT);

grid(ax6,'on');
box(ax6,'on');
xlabel(ax6,'R scale, \rho', 'FontWeight','bold', 'FontSize',FS_LABEL);
ylabel(ax6,'max |\theta| (rad)', 'FontWeight','bold', 'FontSize',FS_LABEL);
title(ax6,'Pitch validity check', 'FontWeight','bold', 'FontSize',FS_TITLE);
format_axes(ax6, FS_AXES);

exportgraphics(fig2, fullfile('PVTOL_Figures','09b_lqr_R_tradeoff_summary.png'), ...
    'Resolution',900, ...
    'BackgroundColor','white');

fprintf('Saved: PVTOL_Figures/09b_lqr_R_tradeoff_summary.png\n');

%% Final summary
fprintf('\n=== R-sweep summary ===\n');
fprintf('Nominal rho = 1 gives max|K| = %.3f, max|u1| = %.4f N, max|theta| = %.4f rad, final error = %.4f m.\n', ...
    results(results(:,1)==1,2), ...
    results(results(:,1)==1,3), ...
    results(results(:,1)==1,4), ...
    results(results(:,1)==1,5));
fprintf('Lower rho gives slightly lower error but much higher gain and control effort.\n');
fprintf('Higher rho reduces control effort but increases final tracking error.\n');

%% ========================================================================
% Local controller used for sensitivity runs
% ========================================================================
function u = sensitivity_controller(t_sample, y_measured, K, L, Gd, Hd, C, ...
                                    m, g, Ts, WN)

    persistent z_hat fx fy started t_prev
    global OBS_LOG

    new_run = isempty(started) || isempty(t_prev) || (t_sample < t_prev);

    if new_run
        started = false;
        z_hat = [];
        fx = [];
        fy = [];
        OBS_LOG = [];
    end

    %% Resolve measurement
    y_measured = y_measured(:);

    if numel(y_measured) == size(C,2)
        y = C*y_measured;
    else
        y = y_measured;
    end

    %% Initialise estimate and prefilter
    if ~started
        z_hat = pinv(C)*y;

        r0 = reference_signal(t_sample);
        fx = [r0(1); 0];
        fy = [r0(2); 0];

        started = true;
    end

    %% Reference prefilter
    r_cmd = reference_signal(t_sample);

    fx = local_prefilter(fx, r_cmd(1), WN, Ts);
    fy = local_prefilter(fy, r_cmd(2), WN, Ts);

    z_ref = [fx(1); fx(2); fy(1); fy(2); 0; 0];

    %% LQR feedback using estimated state
    u_dev = -K*(z_hat - z_ref);

    %% Log current estimate
    OBS_LOG(end+1,:) = [t_sample, z_hat.'];

    %% Observer update
    z_hat = (Gd - L*C)*z_hat + Hd*u_dev + L*y;

    %% Convert deviation input to absolute plant input
    u = u_dev;
    u(2) = u(2) + m*g;

    t_prev = t_sample;
end

%% ========================================================================
% Local critically damped reference prefilter
% ========================================================================
function s = local_prefilter(s, cmd, wn, Ts)
    p = s(1);
    v = s(2);

    a = wn^2*(cmd - p) - 2*wn*v;

    v = v + a*Ts;
    p = p + v*Ts;

    s = [p; v];
end

%% ========================================================================
% Local axes formatting
% ========================================================================
function format_axes(ax, fs)
    set(ax, ...
        'FontSize',fs, ...
        'FontWeight','bold', ...
        'LineWidth',2.4, ...
        'TickDir','out', ...
        'GridAlpha',0.35, ...
        'MinorGridAlpha',0.22, ...
        'Box','on');
end

% Save image inside PVTOL_Figures folder
outDir = 'PVTOL_Figures';

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

exportgraphics(fig2, fullfile(outDir, '09b_lqr_R_tradeoff_summary.png'), ...
    'Resolution', 900, ...
    'BackgroundColor', 'white');

fprintf('Saved: %s\n', fullfile(outDir, '09b_lqr_R_tradeoff_summary.png'));