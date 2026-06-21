function Animate_VTOL(t, x_state, r)
% ANIMATE_VTOL  Animate the planar VTOL aircraft with a tracking camera.
%
% Inputs:
%   t        -- N x 1 time vector from the simulator
%   x_state  -- N x 6 state matrix [x, xdot, y, ydot, theta, thetadot]
%   r        -- thrust offset (used for body geometry scaling)
% =========================================================================

    %% Extract pose -------------------------------------------------------
    pos_x = x_state(:, 1);
    pos_y = x_state(:, 3);
    theta = x_state(:, 5);

    %% Set up figure ------------------------------------------------------
    fig = figure('Name', 'VTOL Animation', 'Color', 'w');
    ax  = axes('Parent', fig);
    hold(ax, 'on');  grid(ax, 'on');  axis(ax, 'equal');
    xlabel(ax, 'X Position (m)');
    ylabel(ax, 'Y Position (m)');

    win = 3;   % half-width of tracking window [m]

    %% Initialise graphic objects once (performance) ----------------------
    h_trail = plot(ax, pos_x(1), pos_y(1), 'b--', 'LineWidth', 1.5);
    h_body  = plot(ax, [0 0], [0 0], 'k', 'LineWidth', 4);
    h_mast  = plot(ax, [0 0], [0 0], 'k', 'LineWidth', 4);
    h_cg    = plot(ax,  0,    0,    'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');

    %% Frame skipping for ~30 FPS regardless of dt -----------------------
    dt         = mean(diff(t));
    target_fps = 30;
    step       = max(1, round(1 / (target_fps * dt)));

    %% Animation loop -----------------------------------------------------
    for i = 1:step:length(t)
        if ~isgraphics(fig),  break;  end    % exit gracefully if user closes

        cx = pos_x(i);  cy = pos_y(i);  th = theta(i);

        % Body endpoints (left and right rotor positions)
        left_x  = cx - r*cos(th);   left_y  = cy - r*sin(th);
        right_x = cx + r*cos(th);   right_y = cy + r*sin(th);

        % Mast endpoint (perpendicular up indicator)
        mast_x = cx - r*sin(th);    mast_y = cy + r*cos(th);

        % Update graphic objects in place
        set(h_trail, 'XData', pos_x(1:i),     'YData', pos_y(1:i));
        set(h_body,  'XData', [left_x right_x], 'YData', [left_y right_y]);
        set(h_mast,  'XData', [cx mast_x],      'YData', [cy mast_y]);
        set(h_cg,    'XData', cx,                'YData', cy);

        % Tracking camera and time-stamped title
        axis(ax, [cx-win, cx+win, cy-win, cy+win]);
        title(ax, sprintf('VTOL Nonlinear Dynamics  |  t = %.2f s', t(i)));

        drawnow;
    end
end
