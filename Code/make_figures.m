function make_figures(T, Y, U, outDir)
% MAKE_FIGURES  Generate LaTeX-friendly PVTOL figures.
%
% This version is optimised for two-column LaTeX reports:
%   - no internal graph titles
%   - large axis fonts for readability after shrinking
%   - bold, thick plot lines
%   - 300 DPI export to prevent LaTeX/Overleaf lag
%   - moderate figure canvas sizes
%
% Inputs:
%   T       : N x 1 time vector
%   Y       : N x 6 true state matrix
%             [x, xdot, y, ydot, theta, thetadot]
%   U       : N x 2 physical input matrix
%             [u1, u2]
%   outDir  : output folder
%
% Example:
%   make_figures(T, Y, U)
%   make_figures(T, Y, U, 'PVTOL_Figures')

    if nargin < 4 || isempty(outDir)
        outDir = 'PVTOL_Figures';
    end

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    %% Load design values -------------------------------------------------
    S = load('designValues.mat');

    Gd = S.Gd;
    Hd = S.Hd;
    L  = S.L;
    C  = S.C;
    K  = S.K;

    e_LQR = S.e_LQR;
    e_obs = S.e_obs;

    WN = S.WN;
    Ac = S.A;
    Bc = S.B;

    THETA_LIN_LIMIT = 0.35;
    if isfield(S,'THETA_VALID') && S.THETA_VALID > 0
        THETA_LIN_LIMIT = S.THETA_VALID;
    end

    SystemParameters
    % brings m, g, Ts, x0, r into scope

    %% ==============================================================
    % TWO-COLUMN LATEX PLOT STYLE
    % ==============================================================
    % Titles are removed because captions in LaTeX already identify
    % the figure. Axis fonts and line widths are deliberately large
    % because the figures will be shrunk in a two-column layout.
    % ==============================================================

    style.LW_MAIN   = 4.2;
    style.LW_CMP    = 3.8;
    style.LW_AUX    = 3.0;
    style.LW_POLE   = 4.2;
    style.LW_PATH   = 4.4;
    style.LW_BODY   = 4.6;

    style.FS_AXES   = 18;
    style.FS_LABEL  = 20;
    style.FS_TITLE  = 1;       % effectively unused
    style.FS_LEGEND = 16;

    style.EXPORT_DPI = 300;

    set(groot,'defaultLineLineWidth',style.LW_MAIN);
    set(groot,'defaultAxesFontSize',style.FS_AXES);
    set(groot,'defaultAxesFontWeight','bold');
    set(groot,'defaultAxesLineWidth',1.8);
    set(groot,'defaultAxesTickDir','out');
    set(groot,'defaultAxesBox','on');

    set(groot,'defaultTextFontSize',style.FS_AXES);
    set(groot,'defaultTextFontWeight','bold');
    set(groot,'defaultLegendFontSize',style.FS_LEGEND);
    set(groot,'defaultLegendFontWeight','bold');

    set(groot,'defaultAxesGridAlpha',0.30);
    set(groot,'defaultAxesMinorGridAlpha',0.18);

    %% Control-rate grid --------------------------------------------------
    tg = (0:Ts:T(end)).';

    %% Commanded reference on Stage-2 time grid ---------------------------
    ref_state = zeros(numel(T), 6);

    for ii = 1:numel(T)
        rc = reference_signal(T(ii));
        ref_state(ii,1) = rc(1);
        ref_state(ii,3) = rc(2);
    end

    %% Auxiliary simulations ----------------------------------------------
    [Y1, U1] = sim_statefb_nl(x0, tg, K, m, g, WN);
    YL       = sim_linear(x0, tg, Gd, Hd, K, WN);

    %% ==============================================================
    % FIGURE 1 -- STAGE 1 FULL-STATE LQR
    % ==============================================================

    fig1 = newReportFigure('Stage 1: Full-State LQR');
    tl1 = plot8(fig1, tg, Y1, U1, m*g, [], '', style);
    savePlot(tl1, outDir, '01_stage_1_state_feedback.png', style);

    fig1a = newReportFigure('Stage 1 States');
    tl1a = plotStates6(fig1a, tg, Y1, [], '', style);
    savePlot(tl1a, outDir, '01a_stage_1_states_only.png', style);

    fig1b = newWideFigure('Stage 1 Inputs');
    tl1b = plotInputs2(fig1b, tg, U1, m*g, style);
    savePlot(tl1b, outDir, '01b_stage_1_inputs_only.png', style);

    %% ==============================================================
    % FIGURE 2 -- LINEAR VS NONLINEAR
    % ==============================================================

    fig2 = newReportFigure('Linear vs Nonlinear');
    tl2 = plot8(fig2, tg, Y1, U1, m*g, YL, 'linear', style);
    savePlot(tl2, outDir, '02_linear_vs_nonlinear.png', style);

    fig2a = newReportFigure('Linear vs Nonlinear States');
    tl2a = plotStates6(fig2a, tg, Y1, YL, 'linear', style);
    savePlot(tl2a, outDir, '02a_linear_vs_nonlinear_states_only.png', style);

    %% ==============================================================
    % FIGURE 3 -- OUTPUT FEEDBACK
    % ==============================================================

    fig3 = newReportFigure('Output Feedback');
    tl3 = plot8(fig3, T, Y, U, m*g, ref_state, 'reference', style);
    savePlot(tl3, outDir, '03_stage_2_output_feedback.png', style);

    fig3a = newReportFigure('Output Feedback States');
    tl3a = plotStates6(fig3a, T, Y, ref_state, 'reference', style);
    savePlot(tl3a, outDir, '03a_output_feedback_states_only.png', style);

    fig3b = newWideFigure('Output Feedback Inputs');
    tl3b = plotInputs2(fig3b, T, U, m*g, style);
    savePlot(tl3b, outDir, '03b_output_feedback_inputs_only.png', style);

    %% ==============================================================
    % FIGURE 4 -- POLE MAP
    % ==============================================================

    fig4 = newWideFigure('Pole Map');
    tl4 = plotPoleMap(fig4, Gd, e_LQR, e_obs, style);
    savePlot(tl4, outDir, '04_discrete_time_pole_map.png', style);

    %% ==============================================================
    % FIGURE 5 -- OBSERVER ESTIMATION
    % ==============================================================

    [Yg, Zhat] = reconstructObserverStates(T, Y, U, tg, Gd, Hd, L, C, m, g);

    fig5 = newReportFigure('Observer Estimation');
    tl5 = plotObserver6(fig5, tg, Yg, Zhat, style);
    savePlot(tl5, outDir, '05_kalman_observer_estimation.png', style);

    %% ==============================================================
    % FIGURE 6 -- 3D FLIGHT PATH
    % ==============================================================

    fig6 = newWideFigure('3D Flight Path');
    ax6 = plot3D_path(fig6, T, Y, r, style);
    savePlot(ax6, outDir, '06_3d_flight_path.png', style);

    %% ==============================================================
    % FIGURE 7 -- LINEARISATION VALIDITY
    % ==============================================================

    fig7 = newReportFigure('Linearisation Validity Limit');
    tl7 = plotLinLimit(fig7, K, Gd, Hd, m, g, Ts, THETA_LIN_LIMIT, style);
    savePlot(tl7, outDir, '07_linearisation_validity_limit.png', style);

    %% ==============================================================
    % FIGURE 8 -- SAMPLING-RATE SENSITIVITY
    % ==============================================================

    fig8 = newWideFigure('Sampling Rate Sensitivity');
    tl8 = plotSampleRate(fig8, Ac, Bc, C, K, style);
    savePlot(tl8, outDir, '08_sampling_rate_sensitivity.png', style);

    fprintf('\nAll LaTeX-friendly figures saved in folder: %s\n', outDir);
end


% ===================================================================
% FIGURE CONSTRUCTORS
% ===================================================================

function fig = newReportFigure(name)
    fig = figure('Name', name, ...
                 'Color','w', ...
                 'Position',[80 80 1350 950], ...
                 'Renderer','painters');
end

function fig = newWideFigure(name)
    fig = figure('Name', name, ...
                 'Color','w', ...
                 'Position',[80 80 1450 760], ...
                 'Renderer','painters');
end


% ===================================================================
% OBSERVER RECONSTRUCTION
% ===================================================================

function [Yg, Zhat] = reconstructObserverStates(T, Y, U, tg, Gd, Hd, L, C, m, g)
    Yg = interp1(T, Y, tg, 'linear', 'extrap');
    Ug = interp1(T, U, tg, 'previous', 'extrap');

    Ng = numel(tg);
    Zhat = zeros(Ng,6);

    zh = pinv(C) * (C * Yg(1,:).');

    for k = 1:Ng
        Zhat(k,:) = zh.';

        u_dev = Ug(k,:).';
        u_dev(2) = u_dev(2) - m*g;

        zh = (Gd - L*C)*zh + Hd*u_dev + L*(C*Yg(k,:).');
    end
end


% ===================================================================
% NONLINEAR STATE-FEEDBACK SIMULATION
% ===================================================================

function [Yout, Uout] = sim_statefb_nl(x0, tg, K, m, g, WN)
    N  = numel(tg);
    Ts = tg(2) - tg(1);

    Yout = zeros(N,6);
    Uout = zeros(N,2);

    z = x0(:);

    r0 = reference_signal(tg(1));
    fx = [r0(1); 0];
    fy = [r0(2); 0];

    for k = 1:N
        r = reference_signal(tg(k));

        fx = prefilter(fx, r(1), WN, Ts);
        fy = prefilter(fy, r(2), WN, Ts);

        xr = [fx(1); fx(2); fy(1); fy(2); 0; 0];

        ud = -K*(z - xr);

        u = ud;
        u(2) = u(2) + m*g;

        Yout(k,:) = z.';
        Uout(k,:) = u.';

        f1 = VTOL_Dynamics_corrected([], z,           u);
        f2 = VTOL_Dynamics_corrected([], z + Ts/2*f1, u);
        f3 = VTOL_Dynamics_corrected([], z + Ts/2*f2, u);
        f4 = VTOL_Dynamics_corrected([], z + Ts*f3,   u);

        z = z + Ts/6*(f1 + 2*f2 + 2*f3 + f4);
    end
end


% ===================================================================
% LINEAR STATE-FEEDBACK SIMULATION
% ===================================================================

function Yout = sim_linear(x0, tg, Gd, Hd, K, WN)
    N  = numel(tg);
    Ts = tg(2) - tg(1);

    Yout = zeros(N,6);
    xl = x0(:);

    r0 = reference_signal(tg(1));
    fx = [r0(1); 0];
    fy = [r0(2); 0];

    for k = 1:N
        r = reference_signal(tg(k));

        fx = prefilter(fx, r(1), WN, Ts);
        fy = prefilter(fy, r(2), WN, Ts);

        xr = [fx(1); fx(2); fy(1); fy(2); 0; 0];

        ud = -K*(xl - xr);

        Yout(k,:) = xl.';

        xl = Gd*xl + Hd*ud;
    end
end


% ===================================================================
% REFERENCE PREFILTER
% ===================================================================

function s = prefilter(s, cmd, wn, Ts)
    p = s(1);
    v = s(2);

    a = wn^2*(cmd - p) - 2*wn*v;

    v = v + a*Ts;
    p = p + v*Ts;

    s = [p; v];
end


% ===================================================================
% 8-PANEL STATE + INPUT PLOT
% ===================================================================

function tl = plot8(fig, t, X, Uc, mg, Xcmp, cmpName, style)
    tl = tiledlayout(fig, 4, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    lab = {'x (m)', ...
           'x vel (m/s)', ...
           'y (m)', ...
           'y vel (m/s)', ...
           '\theta (rad)', ...
           '\theta rate (rad/s)'};

    hasCmp = ~isempty(Xcmp);

    for k = 1:6
        ax = nexttile(tl);

        plot(ax, t, X(:,k), 'b-', ...
             'LineWidth',style.LW_MAIN);

        hold(ax,'on');

        if hasCmp
            plot(ax, t, Xcmp(:,k), 'r--', ...
                 'LineWidth',style.LW_CMP);
        end

        ylabel(ax, lab{k}, ...
               'FontWeight','bold', ...
               'FontSize',style.FS_LABEL);

        grid(ax,'on');
        box(ax,'on');

        setReportAxes(ax, style);

        if k >= 5
            xlabel(ax,'time (s)', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LABEL);
        end

        if k == 1 && hasCmp
            legend(ax,'actual', cmpName, ...
                   'Location','best', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LEGEND);
        end
    end

    ax7 = nexttile(tl);

    plot(ax7, t, Uc(:,1), 'k-', ...
         'LineWidth',style.LW_MAIN);

    grid(ax7,'on');
    box(ax7,'on');
    setReportAxes(ax7, style);

    ylabel(ax7,'u_1 (N)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    xlabel(ax7,'time (s)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ax8 = nexttile(tl);

    plot(ax8, t, Uc(:,2), 'k-', ...
         'LineWidth',style.LW_MAIN);

    hold(ax8,'on');

    yline(ax8, mg, 'r--', ...
          'LineWidth',style.LW_AUX, ...
          'Label','mg', ...
          'FontWeight','bold', ...
          'FontSize',style.FS_LEGEND);

    grid(ax8,'on');
    box(ax8,'on');
    setReportAxes(ax8, style);

    ylabel(ax8,'u_2 (N)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    xlabel(ax8,'time (s)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);
end


% ===================================================================
% 6-PANEL STATE-ONLY PLOT
% ===================================================================

function tl = plotStates6(fig, t, X, Xcmp, cmpName, style)
    tl = tiledlayout(fig, 3, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    lab = {'x (m)', ...
           'x vel (m/s)', ...
           'y (m)', ...
           'y vel (m/s)', ...
           '\theta (rad)', ...
           '\theta rate (rad/s)'};

    hasCmp = ~isempty(Xcmp);

    for k = 1:6
        ax = nexttile(tl);

        plot(ax, t, X(:,k), 'b-', ...
             'LineWidth',style.LW_MAIN);

        hold(ax,'on');

        if hasCmp
            plot(ax, t, Xcmp(:,k), 'r--', ...
                 'LineWidth',style.LW_CMP);
        end

        ylabel(ax, lab{k}, ...
               'FontWeight','bold', ...
               'FontSize',style.FS_LABEL);

        grid(ax,'on');
        box(ax,'on');
        setReportAxes(ax, style);

        if k >= 5
            xlabel(ax,'time (s)', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LABEL);
        end

        if k == 1 && hasCmp
            legend(ax,'actual', cmpName, ...
                   'Location','best', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LEGEND);
        end
    end
end


% ===================================================================
% 2-PANEL INPUT-ONLY PLOT
% ===================================================================

function tl = plotInputs2(fig, t, Uc, mg, style)
    tl = tiledlayout(fig, 1, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    ax1 = nexttile(tl);

    plot(ax1, t, Uc(:,1), 'k-', ...
         'LineWidth',style.LW_MAIN);

    grid(ax1,'on');
    box(ax1,'on');
    setReportAxes(ax1, style);

    xlabel(ax1,'time (s)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(ax1,'u_1 (N)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ax2 = nexttile(tl);

    plot(ax2, t, Uc(:,2), 'k-', ...
         'LineWidth',style.LW_MAIN);

    hold(ax2,'on');

    yline(ax2, mg, 'r--', ...
          'LineWidth',style.LW_AUX, ...
          'Label','mg', ...
          'FontWeight','bold', ...
          'FontSize',style.FS_LEGEND);

    grid(ax2,'on');
    box(ax2,'on');
    setReportAxes(ax2, style);

    xlabel(ax2,'time (s)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(ax2,'u_2 (N)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);
end


% ===================================================================
% OBSERVER PLOT
% ===================================================================

function tl = plotObserver6(fig, tg, Yg, Zhat, style)
    tl = tiledlayout(fig, 3, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    olab = {'x (m)', ...
            'x velocity (m/s)', ...
            'y (m)', ...
            'y velocity (m/s)', ...
            '\theta (rad)', ...
            '\theta rate (rad/s)'};

    for k = 1:6
        ax = nexttile(tl);

        plot(ax, tg, Yg(:,k), 'b-', ...
             'LineWidth',style.LW_MAIN);

        hold(ax,'on');

        plot(ax, tg, Zhat(:,k), 'r--', ...
             'LineWidth',style.LW_CMP);

        ylabel(ax, olab{k}, ...
               'FontWeight','bold', ...
               'FontSize',style.FS_LABEL);

        grid(ax,'on');
        box(ax,'on');
        setReportAxes(ax, style);

        if k == 1
            legend(ax,'true','estimate', ...
                   'Location','best', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LEGEND);
        end

        if k >= 5
            xlabel(ax,'time (s)', ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LABEL);
        end
    end
end


% ===================================================================
% POLE MAP
% ===================================================================

function tl = plotPoleMap(fig, Gd, e_LQR, e_obs, style)
    e_ol = eig(Gd);
    ang  = linspace(0, 2*pi, 400);

    tl = tiledlayout(fig, 1, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    axA = nexttile(tl);
    hold(axA,'on');
    grid(axA,'on');
    axis(axA,'equal');
    box(axA,'on');
    setReportAxes(axA, style);

    fill(axA, cos(ang), sin(ang), [0.88 0.96 0.93], ...
         'EdgeColor','none', ...
         'HandleVisibility','off');

    plot(axA, cos(ang), sin(ang), 'k-', ...
         'LineWidth',style.LW_AUX, ...
         'HandleVisibility','off');

    plot(axA,[-1.3 1.3],[0 0],'-', ...
         'Color',[0.7 0.7 0.7], ...
         'LineWidth',1.4, ...
         'HandleVisibility','off');

    plot(axA,[0 0],[-1.3 1.3],'-', ...
         'Color',[0.7 0.7 0.7], ...
         'LineWidth',1.4, ...
         'HandleVisibility','off');

    drawPoles(axA, e_ol, e_LQR, e_obs, style);

    xlabel(axA,'Re(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(axA,'Im(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    legend(axA,'Location','southoutside', ...
           'FontSize',style.FS_LEGEND, ...
           'FontWeight','bold', ...
           'NumColumns',3);

    xlim(axA,[-1.25 1.25]);
    ylim(axA,[-1.25 1.25]);

    axB = nexttile(tl);
    hold(axB,'on');
    grid(axB,'on');
    box(axB,'on');
    axis(axB,'equal');
    setReportAxes(axB, style);

    fill(axB, cos(ang), sin(ang), [0.88 0.96 0.93], ...
         'EdgeColor','none', ...
         'HandleVisibility','off');

    plot(axB, cos(ang), sin(ang), 'k-', ...
         'LineWidth',style.LW_AUX, ...
         'HandleVisibility','off');

    drawPoles(axB, e_ol, e_LQR, e_obs, style);

    xlabel(axB,'Re(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(axB,'Im(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    xlim(axB,[0.90 1.02]);
    ylim(axB,[-0.06 0.06]);
end


% ===================================================================
% DRAW POLES
% ===================================================================

function drawPoles(ax, e_ol, e_LQR, e_obs, style)
    plot(ax, real(e_ol), imag(e_ol), '+', ...
         'Color',[0.64 0.18 0.18], ...
         'MarkerSize',18, ...
         'LineWidth',style.LW_POLE, ...
         'DisplayName','open-loop');

    plot(ax, real(e_LQR), imag(e_LQR), 'x', ...
         'Color',[0.06 0.43 0.34], ...
         'MarkerSize',18, ...
         'LineWidth',style.LW_POLE, ...
         'DisplayName','LQR');

    plot(ax, real(e_obs), imag(e_obs), 'o', ...
         'Color',[0.10 0.37 0.65], ...
         'MarkerSize',13, ...
         'LineWidth',style.LW_POLE, ...
         'DisplayName','observer');
end


% ===================================================================
% 3D FLIGHT PATH
% ===================================================================

function ax = plot3D_path(fig, T, Y, r, style)
    ax = axes('Parent', fig);

    hold(ax,'on');
    grid(ax,'on');
    box(ax,'on');

    daspect(ax,[1 1 1]);
    axis(ax,'vis3d');
    set(ax,'Projection','perspective');
    setReportAxes(ax, style);

    view(ax, -40, 22);

    xlabel(ax,'X (m)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(ax,'Depth (m)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    zlabel(ax,'Altitude (m)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    px = Y(:,1);
    pz = Y(:,3);
    th = Y(:,5);

    N = numel(T);

    gx = [min(px)-3, max(px)+3];
    [GX,GY] = meshgrid(linspace(gx(1),gx(2),2), [-2 2]);

    surf(ax, GX, GY, zeros(size(GX)), ...
         'FaceColor',[0.85 0.88 0.85], ...
         'EdgeColor','none', ...
         'FaceAlpha',0.45);

    Xs = [px.'; px.'];
    Ys = zeros(2, N);
    Zs = [pz.'; zeros(1,N)];
    Cs = [T(:).'; T(:).'];

    surf(ax, Xs, Ys, Zs, Cs, ...
         'EdgeColor','none', ...
         'FaceAlpha',0.25);

    patch(ax, ...
          'XData',[px; NaN], ...
          'YData',[zeros(N,1); NaN], ...
          'ZData',[pz; NaN], ...
          'CData',[T(:); NaN], ...
          'FaceColor','none', ...
          'EdgeColor','interp', ...
          'LineWidth',style.LW_PATH);

    colormap(ax,'parula');
    set(ax,'CLim',[T(1) T(end)]);

    cb = colorbar(ax);
    cb.Label.String = 'time (s)';
    cb.Label.FontWeight = 'bold';
    cb.Label.FontSize = style.FS_LABEL;
    cb.FontWeight = 'bold';
    cb.FontSize = style.FS_LEGEND;

    plot3(ax, px, zeros(N,1), zeros(N,1), '--', ...
          'Color',[0.50 0.50 0.50], ...
          'LineWidth',style.LW_AUX);

    nStrobe = 8;
    idx = round(linspace(1, N, nStrobe));

    for s = 1:nStrobe
        i  = idx(s);

        cx = px(i);
        cz = pz(i);
        a  = th(i);

        Lx = cx - r*cos(a);
        Lz = cz - r*sin(a);

        Rx = cx + r*cos(a);
        Rz = cz + r*sin(a);

        Mx = cx - 1.2*r*sin(a);
        Mz = cz + 1.2*r*cos(a);

        d = 0.5*r;

        shade = 0.55 - 0.35*(s/nStrobe);
        col = [shade shade shade];

        plot3(ax,[Lx Rx],[0 0],[Lz Rz],'-', ...
              'Color',col, ...
              'LineWidth',style.LW_BODY);

        plot3(ax,[cx Mx],[0 0],[cz Mz],'-', ...
              'Color',col, ...
              'LineWidth',style.LW_BODY - 0.5);

        plot3(ax,[Lx-d*cos(a) Lx+d*cos(a)],[0 0], ...
                 [Lz-d*sin(a) Lz+d*sin(a)],'-', ...
              'Color',col, ...
              'LineWidth',style.LW_BODY - 0.5);

        plot3(ax,[Rx-d*cos(a) Rx+d*cos(a)],[0 0], ...
                 [Rz-d*sin(a) Rz+d*sin(a)],'-', ...
              'Color',col, ...
              'LineWidth',style.LW_BODY - 0.5);
    end

    plot3(ax, px(1), 0, pz(1), 'o', ...
          'MarkerSize',13, ...
          'MarkerFaceColor',[0.10 0.60 0.20], ...
          'MarkerEdgeColor','k', ...
          'LineWidth',2.0);

    plot3(ax, px(end), 0, pz(end), 's', ...
          'MarkerSize',13, ...
          'MarkerFaceColor',[0.85 0.20 0.20], ...
          'MarkerEdgeColor','k', ...
          'LineWidth',2.0);

    text(ax, px(1), 0, pz(1), '  start', ...
         'FontSize',style.FS_LEGEND, ...
         'FontWeight','bold', ...
         'Color',[0.06 0.40 0.13]);

    text(ax, px(end), 0, pz(end), '  end', ...
         'FontSize',style.FS_LEGEND, ...
         'FontWeight','bold', ...
         'Color',[0.55 0.10 0.10]);

    camlight(ax,'headlight');
    lighting(ax,'gouraud');
end


% ===================================================================
% LINEARISATION VALIDITY LIMIT
% ===================================================================

function tl = plotLinLimit(fig, K, Gd, Hd, m, g, Ts, THETA_VALID, style)
    tg = (0:Ts:30).';

    th0_list = [0.20 0.60 1.00];

    fprintf('\n=== Linearisation-validity sweep (regulation) ===\n');
    fprintf('%-11s | %-11s | %-15s | %s\n', ...
            'theta0[rad]','max|theta|','max state err','verdict');
    fprintf('%s\n', repmat('-',1,60));

    Acase = [];
    Bcase = [];

    for ii = 1:numel(th0_list)
        th0 = th0_list(ii);
        x0  = [10; 0; 2; 0; th0; 0];

        Ynl = sim_nl_reg(x0, tg, K, m, g);
        Yl  = sim_lin_reg(x0, tg, Gd, Hd, K);

        e  = Ynl(:,[1 3 5]) - Yl(:,[1 3 5]);
        mE = max(sqrt(sum(e.^2,2)));
        mT = max(abs(Ynl(:,5)));

        if mT <= THETA_VALID
            vd = 'WITHIN  (valid)';
        else
            vd = sprintf('OUTSIDE (> %.2f rad)', THETA_VALID);
        end

        fprintf('%-11.2f | %-11.4f | %-15.4f | %s\n', ...
                th0, mT, mE, vd);

        if ii == 1
            Acase = struct('nl',Ynl,'lin',Yl,'th0',th0);
        end

        if ii == numel(th0_list)
            Bcase = struct('nl',Ynl,'lin',Yl,'th0',th0);
        end
    end

    tl = tiledlayout(fig, 2, 3, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    lab = {'\theta (rad)','x (m)','y (m)'};
    idx = [5 1 3];

    cases = {Acase, Bcase};

    for rr = 1:2
        D = cases{rr};

        for cc = 1:3
            ax = nexttile(tl);

            plot(ax, tg, D.nl(:,idx(cc)), 'b-', ...
                 'LineWidth',style.LW_MAIN);

            hold(ax,'on');

            plot(ax, tg, D.lin(:,idx(cc)), 'r--', ...
                 'LineWidth',style.LW_CMP);

            grid(ax,'on');
            box(ax,'on');
            setReportAxes(ax, style);

            ylabel(ax, lab{cc}, ...
                   'FontWeight','bold', ...
                   'FontSize',style.FS_LABEL);

            if rr == 2
                xlabel(ax,'time (s)', ...
                       'FontWeight','bold', ...
                       'FontSize',style.FS_LABEL);
            end

            if rr == 1 && cc == 3
                legend(ax,'nonlinear','linear', ...
                       'Location','best', ...
                       'FontWeight','bold', ...
                       'FontSize',style.FS_LEGEND);
            end
        end
    end
end


% ===================================================================
% SAMPLING-RATE SENSITIVITY
% ===================================================================

function tl = plotSampleRate(fig, Ac, Bc, C, K, style)
    sysc = ss(Ac, Bc, C, zeros(size(C,1), size(Bc,2)));

    Ts_list = [0.005 0.01 0.02 0.05 0.10 0.15 0.20 0.30];

    maxz   = nan(size(Ts_list));
    polesC = cell(size(Ts_list));

    fprintf('\n=== Sampling-rate sensitivity (fixed nominal K) ===\n');
    fprintf('%-9s | %-9s | %-11s | %s\n', ...
            'Ts[s]','rate[Hz]','max|pole|','verdict');
    fprintf('%s\n', repmat('-',1,52));

    for i = 1:numel(Ts_list)
        Ts = Ts_list(i);

        sysd = c2d(sysc, Ts, 'zoh');
        ev   = eig(sysd.A - sysd.B*K);
        mz   = max(abs(ev));

        maxz(i)   = mz;
        polesC{i} = ev;

        if mz < 1
            v = 'STABLE';
        else
            v = 'UNSTABLE';
        end

        fprintf('%-9.3f | %-9.0f | %-11.4f | %s\n', ...
                Ts, 1/Ts, mz, v);
    end

    tl = tiledlayout(fig, 1, 2, ...
        'Padding','compact', ...
        'TileSpacing','compact');

    ax1 = nexttile(tl);

    plot(ax1, Ts_list, maxz, 'o-', ...
         'LineWidth',style.LW_MAIN, ...
         'MarkerFaceColor','b', ...
         'MarkerSize',11);

    hold(ax1,'on');

    plot(ax1, [min(Ts_list) max(Ts_list)], [1 1], 'r--', ...
         'LineWidth',style.LW_CMP);

    grid(ax1,'on');
    box(ax1,'on');
    setReportAxes(ax1, style);

    xlabel(ax1,'sample time T_s (s)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(ax1,'max |pole|', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    legend(ax1,'max |z|','|z| = 1', ...
           'Location','northwest', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LEGEND);

    ax2 = nexttile(tl);

    th = linspace(0,2*pi,240);

    plot(ax2, cos(th), sin(th), 'k-', ...
         'LineWidth',style.LW_AUX);

    hold(ax2,'on');
    axis(ax2,'equal');
    grid(ax2,'on');
    box(ax2,'on');
    setReportAxes(ax2, style);

    cmap = lines(numel(Ts_list));

    for i = 1:numel(Ts_list)
        plot(ax2, real(polesC{i}), imag(polesC{i}), 'x', ...
             'Color',cmap(i,:), ...
             'MarkerSize',14, ...
             'LineWidth',style.LW_CMP);
    end

    xlabel(ax2,'Re(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    ylabel(ax2,'Im(z)', ...
           'FontWeight','bold', ...
           'FontSize',style.FS_LABEL);

    xlim(ax2,[-1.3 1.5]);
    ylim(ax2,[-1.3 1.3]);
end


% ===================================================================
% NONLINEAR REGULATION SIMULATION
% ===================================================================

function Y = sim_nl_reg(x0, tg, K, m, g)
    N  = numel(tg);
    Ts = tg(2) - tg(1);

    Y = zeros(N,6);
    z = x0(:);

    for k = 1:N
        u = -K*z;
        u(2) = u(2) + m*g;

        Y(k,:) = z.';

        f1 = VTOL_Dynamics_corrected([], z,           u);
        f2 = VTOL_Dynamics_corrected([], z + Ts/2*f1, u);
        f3 = VTOL_Dynamics_corrected([], z + Ts/2*f2, u);
        f4 = VTOL_Dynamics_corrected([], z + Ts*f3,   u);

        z = z + Ts/6*(f1 + 2*f2 + 2*f3 + f4);
    end
end


% ===================================================================
% LINEAR REGULATION SIMULATION
% ===================================================================

function Y = sim_lin_reg(x0, tg, Gd, Hd, K)
    N = numel(tg);

    Y  = zeros(N,6);
    xl = x0(:);

    for k = 1:N
        ud = -K*xl;

        Y(k,:) = xl.';

        xl = Gd*xl + Hd*ud;
    end
end


% ===================================================================
% AXES STYLE
% ===================================================================

function setReportAxes(ax, style)
    set(ax, ...
        'FontWeight','bold', ...
        'FontSize',style.FS_AXES, ...
        'LineWidth',1.8, ...
        'GridAlpha',0.30, ...
        'MinorGridAlpha',0.18, ...
        'TickDir','out', ...
        'Box','on');

    ax.XAxis.FontSize = style.FS_AXES;
    ax.YAxis.FontSize = style.FS_AXES;

    if isprop(ax, 'ZAxis')
        ax.ZAxis.FontSize = style.FS_AXES;
    end
end


% ===================================================================
% SAVE PLOT
% ===================================================================

function savePlot(obj, outDir, fileName, style)
    drawnow;

    fullPath = fullfile(outDir, fileName);

    try
        exportgraphics(obj, fullPath, ...
            'Resolution',style.EXPORT_DPI, ...
            'BackgroundColor','white');

        fprintf('Saved: %s\n', fullPath);

    catch ME
        warning('exportgraphics failed for %s.', fileName);
        warning('%s', ME.message);

        fig = ancestor(obj, 'figure');

        if ~isempty(fig)
            try
                frame = getframe(fig);
                imwrite(frame.cdata, fullPath);
                fprintf('Saved using fallback method: %s\n', fullPath);
            catch ME2
                warning('Fallback save failed for %s.', fileName);
                warning('%s', ME2.message);
            end
        end
    end
end