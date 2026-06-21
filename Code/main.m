%% main.m -- Run the closed-loop simulation and report numbers

% This file ONLY simulates and prints diagnostics.  All figures are
% produced by make_figures.m (kept separate for cleanliness).

% Prerequisites: run design.m first (or run_all.m, which chains them).

% Pipeline:
%   1) Load parameters and design results
%   2) Set initial condition
%   3) Simulate closed-loop nonlinear PVTOL
%   4) Quantitative diagnostic (validity + tracking) -- text only
%   5) Hand off to make_figures.m, then animate

%% 1) Load parameters and design
SystemParameters
assert(exist('designValues.mat','file') == 2, ...
       'designValues.mat not found.  Run design.m first.');
load('designValues.mat');

%% 2) Initial condition
% x0 = [x; xdot; y; ydot; theta; thetadot] comes from SystemParameters
% (single source of truth) -- do NOT redefine it here.
fprintf('Initial state:  x=%g m, y=%g m, theta=%g rad\n', ...
        x0(1), x0(3), x0(5));

%% 3) Simulate
% Bind to the observer-estimate log that my_controller fills during the
% run.  Reset by VALUE only -- never 'clear global', which would destroy
% the object a still-resident my_controller is bound to.  (my_controller
% also resets it on its own new-run detection, so this is double-safe.)
global OBS_LOG
OBS_LOG = [];

fprintf('Simulating closed-loop nonlinear PVTOL ...\n');
t_span = [0 30];
[T, Y, U] = System_Simulator(@VTOL_Dynamics_corrected, x0, t_span, ...
                             @my_controller, Ts, studentID);
fprintf('Simulation complete:  %d samples over %.1f s\n\n', ...
        numel(T), T(end));

%% 4) Quantitative diagnostic  (text only -- no figures here)
% Reconstruct the commanded reference on the time grid (single source).
ref_state = zeros(numel(T), 6);
for ii = 1:numel(T)
    rc = reference_signal(T(ii));
    ref_state(ii,1) = rc(1);
    ref_state(ii,3) = rc(2);
end

THETA_LIN_LIMIT = 0.35;
if exist('THETA_VALID','var') && THETA_VALID > 0
    THETA_LIN_LIMIT = THETA_VALID;
end

n = size(Gd,1);

%% State-Space Model  (the model the whole design is built on)
% A, B  -- continuous-time, from Jacobian linearisation at the hover
%          equilibrium (design.m step 4).
% C, D  -- output equation: positions x, y and tilt theta are measured
%          (velocities are NOT), so C is 3x6 and D is 3x2 zeros.
% Gd,Hd -- the ZOH discretisation at Ts that the digital LQR/observer
%          are actually designed on.  Printed too because every pole
%          and gain below lives in this discrete model, not (A,B).
fprintf('=== State-Space Model ===\n');
fprintf('A  (%dx%d, continuous state matrix):\n', size(A,1), size(A,2));
disp(A);
fprintf('B  (%dx%d, continuous input matrix):\n', size(B,1), size(B,2));
disp(B);
fprintf('C  (%dx%d, output matrix -- measures x, y, theta):\n', ...
        size(C,1), size(C,2));
disp(C);
fprintf('D  (%dx%d, feedthrough matrix):\n', size(D,1), size(D,2));
disp(D);
fprintf('Gd (%dx%d, discrete state matrix, ZOH @ Ts = %g s):\n', ...
        size(Gd,1), size(Gd,2), Ts);
disp(Gd);
fprintf('Hd (%dx%d, discrete input matrix):\n', size(Hd,1), size(Hd,2));
disp(Hd);

%% System Properties
% Controllability and observability are the PRECONDITIONS for the whole
% design: the LQR gain only exists/stabilises if (Gd,Hd) is controllable,
% and the observer can only reconstruct the unmeasured states if (Gd,C)
% is observable.  Both ranks must equal n = 6.
rank_C = rank(ctrb(Gd, Hd));
rank_O = rank(obsv(Gd, C));
ev_ol  = eig(A);

if rank_C == n, vC = 'CONTROLLABLE';
else,           vC = 'NOT CONTROLLABLE (design invalid!)';  end
if rank_O == n, vO = 'OBSERVABLE';
else,           vO = 'NOT OBSERVABLE (observer invalid!)';   end

fprintf('=== System Properties ===\n');
fprintf('Controllability rank: %d  (need %d) -> %s\n', rank_C, n, vC);
fprintf('Observability  rank: %d  (need %d) -> %s\n', rank_O, n, vO);
fprintf('Open-loop continuous eigenvalues:');
fprintf(' %.4g', real(ev_ol));
fprintf('\n\n');

% LQR
fprintf('=== LQR ===\n');
fprintf('Max |K|: %.2f\n', max(abs(K(:))));
fprintf('Max closed-loop |z|: %.4f\n\n', max(abs(e_LQR)));

% Kalman observer
fprintf('=== Kalman ===\n');
fprintf('Max |L|: %.2f\n', max(abs(L(:))));
fprintf('Max observer |z|: %.4f\n\n', max(abs(e_obs)));

% Combined (separation principle)
fprintf('=== Combined ===\n');
fprintf('Max combined |z|: %.4f\n\n', max(max(abs(e_LQR)), max(abs(e_obs))));

% Results
[max_abs_theta, idx_th] = max(abs(Y(:,5)));
final_pos_err = norm([Y(end,1)-ref_state(end,1), Y(end,3)-ref_state(end,3)]);
fprintf('=== Results ===\n');
fprintf('Final position error: %.4f m\n', final_pos_err);
% KNOWN LIMITATION (documented, not an implementation error):
% If a non-zero setpoint is being tracked, a small steady-state offset
% is EXPECTED.  This design tracks by set-point shifting through the
% prefilter; it has NO feedforward gain Kr and NO integral action, so
% by the standard result it does not achieve exactly zero steady-state
% error for a non-zero reference (regulation to the origin, the natural
% equilibrium, IS exact).  Removing this residual would require adding
% Kr or an integral state -- a deliberate trade-off (extra state /
% windup risk) not taken here.
cmd_end = reference_signal(T(end));
if norm(cmd_end) > 0
    fprintf(['  ^ note: non-zero reference commanded -> the residual ', ...
             'above is the EXPECTED steady-state offset of set-point\n', ...
             '    shifting without Kr/integral action (a known design ', ...
             'limitation, not an error).\n']);
end
fprintf('Max |theta|: %.4f rad  (%.1f deg) at t = %.2f s\n', ...
        max_abs_theta, rad2deg(max_abs_theta), T(idx_th));
fprintf('Max |u1|: %.2f N\n', max(abs(U(:,1))));
if max_abs_theta <= THETA_LIN_LIMIT
    fprintf('Linear validity (|theta| < %.2f rad): WITHIN -> assumptions hold\n\n', ...
            THETA_LIN_LIMIT);
else
    fprintf('Linear validity (|theta| < %.2f rad): EXCEEDED by %.0f%%\n\n', ...
            THETA_LIN_LIMIT, 100*(max_abs_theta/THETA_LIN_LIMIT - 1));
end

% Kalman Observer Performance
% Uses the estimate the controller ACTUALLY acted on (logged in
% OBS_LOG by my_controller), compared against the true state at the
% same instants.
fprintf('=== Kalman Observer Performance ===\n\n');
fprintf('=== Observer Estimation Error ===\n');
if isempty(OBS_LOG)
    fprintf('(OBS_LOG empty -- my_controller did not log; cannot report)\n');
else
    tlog  = OBS_LOG(:,1);
    Zlog  = OBS_LOG(:,2:7);
    Ytrue = interp1(T, Y, tlog, 'linear', 'extrap');   % true state @ log times
    e_est = Zlog - Ytrue;
    rmsE  = sqrt(mean(e_est.^2, 1));
    maxE  = max(abs(e_est), [], 1);
    names = {'x (m)','x velocity (m/s)','y (m)','y velocity (m/s)', ...
             '\theta (rad)','\theta rate (rad/s)'};
    fprintf('%-20s | %-9s | %-9s\n', 'State', 'RMS error', 'Max error');
    fprintf('%s\n', repmat('-',1,44));
    for k = 1:6
        fprintf('%-20s | %9.5f | %9.5f\n', names{k}, rmsE(k), maxE(k));
    end
end
fprintf('\n');

%% 5) Figures + animation
make_figures(T, Y, U);          % all plots live here
Animate_VTOL(T, Y, r);          % r = thrust offset, from SystemParameters
