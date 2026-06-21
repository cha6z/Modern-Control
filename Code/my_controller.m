function u = my_controller(t_sample, y_measured)
% MY_CONTROLLER  Observer-based LQR with smooth, validity-aware tracking.
%
% Pipeline at each sample instant k:
%   1) Resolve sensed output y = [x; y; theta] (handles full-state OR
%      output being passed by the simulator).
%   2) SMOOTH reference prefilter: pass the raw command
%      reference_signal(t) through a critically-damped 2nd-order filter.
%      Continuous acceleration -> theta stays smooth (no sawtooth), and
%      the filter bandwidth wn is sized from the linear-validity budget
%      so the implied tilt stays inside the validated region.  The
%      filter is seeded at the COMMAND, so when the command is zero the
%      reference is inert and the controller is the plain smooth LQR
%      regulator -- the original behaviour, unchanged.
%   3) Tracking LQR feedback:  u_dev = -K * (z_hat - z_ref)
%   4) Observer update (prediction / Luenberger form).
%   5) Add hover-thrust feedforward (m*g) -> absolute thrust.
%
% Inputs:
%   t_sample   - current simulation time (s); drives the reference
%   y_measured - simulator output: full state (n x 1) or sensed
%                output (p x 1); both handled correctly.
% Output:
%   u (2x1)    - [u1; u2] absolute control input for the plant.


    persistent z_hat K L Gd Hd C m_p g_p Ts_p WN_p fx fy started t_prev
    global OBS_LOG          % [t, z_hat'] log for the command-window report

    %% Detect a fresh run -------------------------------------------------
    % A new simulation restarts the clock, so t_sample jumps back (e.g.
    % 30 -> 0).  This is a reliable "new run" signal that does NOT depend
    % on persistents being cleared, so the controller always re-loads its
    % design and resets the log even on a repeated run in the same session.
    new_run = isempty(K) || isempty(t_prev) || (t_sample < t_prev);

    %% New run: load gains, parameters, prefilter bandwidth, reset log ----
    if new_run
        S  = load('designValues.mat');
        K  = S.K;   L  = S.L;
        Gd = S.Gd;  Hd = S.Hd;  C = S.C;
        WN_p = S.WN;                       % prefilter natural frequency

        SystemParameters;                  % m, g, Ts into scope
        m_p = m;  g_p = g;  Ts_p = Ts;

        started = false;
        OBS_LOG = [];                      % fresh estimate log this run
        fprintf('--- my_controller initialised ---\n');
        fprintf('  K:[%dx%d] L:[%dx%d]  Hover m*g = %.2f N\n', ...
                size(K), size(L), m_p*g_p);
        fprintf('  Smooth reference prefilter: wn = %.3f rad/s (zeta=1)\n', WN_p);
        fprintf('---------------------------------\n\n');
    end

    %% Step 1: resolve the sensed measurement ---------------------------
    y_measured = y_measured(:);
    if numel(y_measured) == size(C, 2)        % full state passed (n x 1)
        y = C * y_measured;                   % -> sensed output (p x 1)
    else                                      % output already passed
        y = y_measured;
    end                                       % y = [x; y; theta]

    %% First call: seed estimate from measurement, prefilter from CMD ---
    % Seeding the prefilter at the command (not the position) means a
    % zero command leaves z_ref = 0, so the initial-offset recovery is
    % the plain smooth LQR regulator (the original, correct behaviour).
    if ~started
        z_hat = pinv(C) * y;                  % min-norm state from y
        r0    = reference_signal(t_sample);
        fx    = [r0(1); 0];                   % [x_ref ; xdot_ref]
        fy    = [r0(2); 0];                   % [y_ref ; ydot_ref]
        started = true;
    end

    %% Step 2: smooth critically-damped reference prefilter ------------
    r  = reference_signal(t_sample);          % raw command [x_ref; y_ref]
    fx = prefilter(fx, r(1), WN_p, Ts_p);
    fy = prefilter(fy, r(2), WN_p, Ts_p);
    z_ref = [fx(1); fx(2); fy(1); fy(2); 0; 0];   % hover orientation

    %% Step 3: tracking LQR feedback (deviation coords) ----------------
    u_dev = -K * (z_hat - z_ref);

    % Log the estimate the controller ACTED ON at this instant, so
    % main.m can report true observer performance (RMS / max error).
    OBS_LOG(end+1, :) = [t_sample, z_hat.'];

    %% Step 4: observer update (prediction / Luenberger form) ---------
    %   z_hat[k+1] = Gd*z_hat + Hd*u_dev + L*(y - C*z_hat)
    z_hat = (Gd - L*C) * z_hat + Hd * u_dev + L * y;

    %% Step 5: convert deviation -> absolute thrust -------------------
    u    = u_dev;
    u(2) = u(2) + m_p*g_p;

    t_prev = t_sample;          % for new-run detection on the next call
end


function s = prefilter(s, cmd, wn, Ts)
% One step of a per-axis critically-damped (zeta = 1) 2nd-order
% reference prefilter.  Continuous acceleration -> SMOOTH reference,
% no corners or chatter.  s = [p_ref ; v_ref].
    p = s(1);  v = s(2);
    a = wn^2*(cmd - p) - 2*wn*v;
    v = v + a*Ts;
    p = p + v*Ts;
    s = [p; v];
end
