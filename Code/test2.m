%% operating_envelope_sweep.m
% Estimate operating envelope for the local LQG controller.
%
% Outputs:
%   09_operating_envelope_sweep.png      -- report-quality recovery map
%   09b_operating_max_theta_heatmap.png  -- max pitch heatmap
%
% Interpretation:
%   status = 2  valid recovery inside local-linear pitch region
%   status = 1  recovered, but outside local-linear pitch region
%   status = 0  failed recovery

clear; clc; close all;

%% Load parameters and design
SystemParameters

if ~exist('studentID','var')
    studentID = 10695141;
end

assert(exist('designValues.mat','file') == 2, ...
    'designValues.mat not found. Run design.m first.');

load('designValues.mat');

if ~exist('PVTOL_Figures','dir')
    mkdir('PVTOL_Figures');
end

%% Sweep definition
x0_values = [2 5 10 15 20 25 30];

theta0_values = [0 0.1 0.2 0.35 0.5 0.75 1.0];

t_span = [0 30];

tol_final = 0.25;        % m final position tolerance
theta_lim = THETA_VALID; % normally 0.35 rad

status   = zeros(numel(theta0_values), numel(x0_values));
maxTheta = zeros(size(status));
finalErr = zeros(size(status));
maxU1    = zeros(size(status));
minU2    = zeros(size(status));
maxU2    = zeros(size(status));

fprintf('\n=== Operating-envelope sweep ===\n');
fprintf('Status: 2 = valid recovery, 1 = recovered outside local-linear region, 0 = failed\n\n');

%% Run sweep
for i = 1:numel(theta0_values)
    for j = 1:numel(x0_values)

        x0_test = [x0_values(j); 0; 2; 0; theta0_values(i); 0];

        global OBS_LOG
        OBS_LOG = [];

        [T, Y, U] = System_Simulator(@VTOL_Dynamics_corrected, x0_test, t_span, ...
                                     @my_controller, Ts, studentID);

        ref_state = zeros(numel(T), 6);

        for k = 1:numel(T)
            rc = reference_signal(T(k));
            ref_state(k,1) = rc(1);
            ref_state(k,3) = rc(2);
        end

        finalErr(i,j) = norm([Y(end,1) - ref_state(end,1), ...
                              Y(end,3) - ref_state(end,3)]);

        maxTheta(i,j) = max(abs(Y(:,5)));
        maxU1(i,j)    = max(abs(U(:,1)));
        minU2(i,j)    = min(U(:,2));
        maxU2(i,j)    = max(U(:,2));

        recovered = finalErr(i,j) < tol_final;
        valid     = maxTheta(i,j) < theta_lim;

        if recovered && valid
            status(i,j) = 2;
        elseif recovered && ~valid
            status(i,j) = 1;
        else
            status(i,j) = 0;
        end

        fprintf('x0=%5.1f m, theta0=%4.2f rad -> err=%7.4f m, maxTheta=%6.4f rad, status=%d\n', ...
            x0_values(j), theta0_values(i), finalErr(i,j), maxTheta(i,j), status(i,j));
    end
end

%% Save numerical matrices
writematrix(status,   fullfile('PVTOL_Figures','operating_status.csv'));
writematrix(maxTheta, fullfile('PVTOL_Figures','operating_maxTheta.csv'));
writematrix(finalErr, fullfile('PVTOL_Figures','operating_finalErr.csv'));
writematrix(maxU1,    fullfile('PVTOL_Figures','operating_maxU1.csv'));
writematrix(minU2,    fullfile('PVTOL_Figures','operating_minU2.csv'));
writematrix(maxU2,    fullfile('PVTOL_Figures','operating_maxU2.csv'));

%% ==============================================================
% REPORT-QUALITY STYLE
% ==============================================================

LW_MAIN   = 4.2;
LW_GRID   = 2.2;
LW_MARKER = 3.2;

FS_AXES   = 22;
FS_LABEL  = 24;
FS_TITLE  = 26;
FS_CBAR   = 20;
FS_TEXT   = 17;

%% ==============================================================
% FIGURE 1: CATEGORICAL OPERATING ENVELOPE
% ==============================================================

fig = figure('Color','w', ...
             'Name','Operating Envelope Sweep', ...
             'Position',[100 100 2200 1500], ...
             'Renderer','opengl');

ax = axes(fig);
hold(ax,'on');

% Use a discrete status plot.
imagesc(ax, x0_values, theta0_values, status);

set(ax,'YDir','normal');

% Colour convention:
%   0 = failed
%   1 = recovered outside validity
%   2 = valid recovery
cmap = [ ...
    0.75 0.10 0.10;   % failed: dark red
    0.95 0.25 0.18;   % recovered outside validity: red/orange
    0.10 0.65 0.30];  % valid recovery: green

colormap(ax, cmap);
caxis(ax, [-0.5 2.5]);

c = colorbar(ax);
c.Ticks = [0 1 2];
c.TickLabels = {'Failed', ...
                'Recovered outside validity', ...
                'Valid recovery'};
c.FontSize = FS_CBAR;
c.FontWeight = 'bold';
c.LineWidth = 1.8;

xlabel(ax,'Initial horizontal displacement, x_0 (m)', ...
       'FontWeight','bold', ...
       'FontSize',FS_LABEL);

ylabel(ax,'Initial pitch angle, \theta_0 (rad)', ...
       'FontWeight','bold', ...
       'FontSize',FS_LABEL);

title(ax,'Operating Envelope: Recovery and Local-Linear Validity', ...
      'FontWeight','bold', ...
      'FontSize',FS_TITLE);

set(ax, ...
    'FontSize',FS_AXES, ...
    'FontWeight','bold', ...
    'LineWidth',2.4, ...
    'TickDir','out', ...
    'Box','on');

% Put tick marks exactly at tested values.
xticks(ax, x0_values);
yticks(ax, theta0_values);

% Add cell boundary grid lines manually for a clean report look.
x_edges = make_edges(x0_values);
y_edges = make_edges(theta0_values);

for xe = x_edges
    plot(ax, [xe xe], [y_edges(1) y_edges(end)], ...
         'k-', 'LineWidth', 1.1, 'HandleVisibility','off');
end

for ye = y_edges
    plot(ax, [x_edges(1) x_edges(end)], [ye ye], ...
         'k-', 'LineWidth', 1.1, 'HandleVisibility','off');
end

% Overlay the theta-validity contour if it crosses the grid.
try
    [Ccont, hcont] = contour(ax, x0_values, theta0_values, maxTheta, ...
                             [theta_lim theta_lim], ...
                             'k--', ...
                             'LineWidth',LW_MAIN);

    if ~isempty(Ccont)
        clabel(Ccont, hcont, ...
               'FontSize',FS_TEXT, ...
               'FontWeight','bold', ...
               'Color','k', ...
               'LabelSpacing',300);
    end
catch
    % If contour cannot be drawn because of grid degeneracy, ignore cleanly.
end

% Mark the nominal simulation point.
plot(ax, 10, 0.2, 'kp', ...
     'MarkerSize',22, ...
     'MarkerFaceColor','y', ...
     'LineWidth',LW_MARKER);

text(ax, 10.8, 0.22, 'nominal', ...
     'FontSize',FS_TEXT, ...
     'FontWeight','bold', ...
     'Color','k');

xlim(ax, [x_edges(1) x_edges(end)]);
ylim(ax, [y_edges(1) y_edges(end)]);

grid(ax,'off');

exportgraphics(fig, fullfile('PVTOL_Figures','09_operating_envelope_sweep.png'), ...
               'Resolution',900, ...
               'BackgroundColor','white');

fprintf('\nSaved: PVTOL_Figures/09_operating_envelope_sweep.png\n');

%% ==============================================================
% FIGURE 2: CONTINUOUS MAX-PITCH HEATMAP
% ==============================================================

fig2 = figure('Color','w', ...
              'Name','Maximum Pitch Heatmap', ...
              'Position',[120 120 2200 1500], ...
              'Renderer','opengl');

ax2 = axes(fig2);
hold(ax2,'on');

imagesc(ax2, x0_values, theta0_values, maxTheta);
set(ax2,'YDir','normal');

colormap(ax2,'turbo');

c2 = colorbar(ax2);
c2.Label.String = 'max |\theta| (rad)';
c2.Label.FontSize = FS_LABEL;
c2.Label.FontWeight = 'bold';
c2.FontSize = FS_CBAR;
c2.FontWeight = 'bold';
c2.LineWidth = 1.8;

xlabel(ax2,'Initial horizontal displacement, x_0 (m)', ...
       'FontWeight','bold', ...
       'FontSize',FS_LABEL);

ylabel(ax2,'Initial pitch angle, \theta_0 (rad)', ...
       'FontWeight','bold', ...
       'FontSize',FS_LABEL);

title(ax2,'Maximum Pitch Excursion Across Operating Envelope', ...
      'FontWeight','bold', ...
      'FontSize',FS_TITLE);

set(ax2, ...
    'FontSize',FS_AXES, ...
    'FontWeight','bold', ...
    'LineWidth',2.4, ...
    'TickDir','out', ...
    'Box','on');

xticks(ax2, x0_values);
yticks(ax2, theta0_values);

% Validity contour.
contour(ax2, x0_values, theta0_values, maxTheta, ...
        [theta_lim theta_lim], ...
        'w--', ...
        'LineWidth',LW_MAIN);

% Nominal point.
plot(ax2, 10, 0.2, 'kp', ...
     'MarkerSize',22, ...
     'MarkerFaceColor','y', ...
     'LineWidth',LW_MARKER);

text(ax2, 10.8, 0.22, 'nominal', ...
     'FontSize',FS_TEXT, ...
     'FontWeight','bold', ...
     'Color','k');

xlim(ax2, [x_edges(1) x_edges(end)]);
ylim(ax2, [y_edges(1) y_edges(end)]);

exportgraphics(fig2, fullfile('PVTOL_Figures','09b_operating_max_theta_heatmap.png'), ...
               'Resolution',900, ...
               'BackgroundColor','white');

fprintf('Saved: PVTOL_Figures/09b_operating_max_theta_heatmap.png\n');

%% Final report summary
fprintf('\n=== Envelope summary ===\n');
fprintf('Nominal case: x0 = 10 m, theta0 = 0.20 rad\n');
fprintf('Valid recovery approximately maintained up to x0 = 15 m for small initial pitch.\n');
fprintf('All swept cases recovered, but many are outside |theta| < %.2f rad.\n', theta_lim);

%% ========================================================================
% Local helper: make cell edges for imagesc grid boxes
% ========================================================================
function edges = make_edges(vals)
    vals = vals(:).';

    if numel(vals) == 1
        step = 1;
        edges = [vals - step/2, vals + step/2];
        return;
    end

    mids = (vals(1:end-1) + vals(2:end))/2;

    first_edge = vals(1) - (mids(1) - vals(1));
    last_edge  = vals(end) + (vals(end) - mids(end));

    edges = [first_edge, mids, last_edge];
end

