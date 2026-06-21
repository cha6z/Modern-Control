%% design.m -- VTOL Controller and Observer Design (EGH445)

% Designs an observer-based LQR controller for the planar VTOL.

% Pipeline (each step in its own section -- point and explain at demo):
%   1)  Load physical parameters
%   2)  Define nonlinear dynamics symbolically
%   3)  Find hover equilibrium and verify f(z_eq, u_eq) = 0
%   4)  Linearise via Jacobians at the equilibrium  ->  (A, B)
%   5)  Define output equation                       ->  C, D
%   6)  Discretise with ZOH                          ->  (Gd, Hd)
%   7)  Verify controllability and observability
%   8)  Design LQR state-feedback gain K  (Bryson's rule)
%   9)  Design Kalman observer gain L     (dual of LQR)
%   10) Save results and produce pole map

% Outputs:
%   designValues.mat  -- contains K, L, Gd, Hd, C, D, A, B, Q, R, Qn, Rn, 
%   Figure 'Pole Map' -- visual stability proof on the z-plane


clear; clc;

%% 1) Physical parameters 
SystemParameters
fprintf('Parameters loaded:  m = %g kg,  J = %g kg.m^2,  r = %g m\n', m, J, r);

%% 2) Nonlinear dynamics (symbolic) 
% State:  z = [x; xdot; y; ydot; theta; thetadot]
%
% Input (per the assignment brief's prescribed substitution):
%   u1 = F1                 (differential / lateral thrust)
%   u2 = F2 - m*g           (vertical thrust MINUS the hover weight)
% So the physical vertical force is  F2 = u2 + m*g.  Substituting the
% brief's definition into the equations of motion absorbs gravity into
% the input variable, which places the hover equilibrium at the ORIGIN
% in BOTH state and input (u_eq = [0; 0]) -- exactly as the brief sets
% up for linearisation.

syms x1 x2 x3 x4 x5 x6 u1 u2 real
z_sym = [x1; x2; x3; x4; x5; x6];
u_sym = [u1; u2];

% F2 = u2 + m*g  is substituted directly below.
f = [ x2;
     -((u2 + m*g)/m)*sin(x5) - (c/m)*x2;
      x4;
      ((u2 + m*g)/m)*cos(x5) - g - (c/m)*x4;
      x6;
      (r/J)*u1 ];

%% 3) Hover equilibrium  f(z_eq, u_eq) = 0 
% Position is arbitrary (translationally invariant), so set to 0.
% Because the brief's substitution u2 = F2 - m*g already removed the
% hover weight from the input, the equilibrium INPUT is now the ORIGIN:
% u_eq = [0; 0]  (physically this is F1 = 0, F2 = m*g -- thrust still
% balances gravity, but in the shifted u2 coordinate that reads as 0).

z_eq = [0; 0; 0; 0; 0; 0];
u_eq = [0; 0];

% Sanity check: f evaluated at the equilibrium must be the zero vector
f_at_eq = double( subs(f, [z_sym; u_sym], [z_eq; u_eq]) );
assert(norm(f_at_eq) < 1e-12, ...
       'Hover equilibrium does not satisfy f(z_eq, u_eq) = 0 -- check dynamics.');
fprintf('Hover equilibrium verified:  ||f(z_eq, u_eq)|| = %.2e\n', norm(f_at_eq));

%% 4) Linearisation via Jacobians 
% A = df/dz |_{eq}    small state deviations from hover
% B = df/du |_{eq}    small input deviations (u already measured from
%                     hover, so u_eq = 0 -- linearise about the origin)

% Hartman-Grobman theorem (lecture L3): provided A has no eigenvalues on the
% imaginary axis, the linear system reproduces nonlinear trajectories near
% the equilibrium.

A_sym = jacobian(f, z_sym);
B_sym = jacobian(f, u_sym);

A = double( subs(A_sym, [z_sym; u_sym], [z_eq; u_eq]) );
B = double( subs(B_sym, [z_sym; u_sym], [z_eq; u_eq]) );

fprintf('\nLinearised continuous-time matrices:\n');
fprintf('A =\n');  disp(A);
fprintf('B =\n');  disp(B);

%% 5) Output equation 
% Sensors measure positions (x, y) and orientation (theta).
% Velocities are NOT measured -- the observer must reconstruct them.

C = [1 0 0 0 0 0;     % x       (state 1)
     0 0 1 0 0 0;     % y       (state 3)
     0 0 0 0 1 0];    % theta   (state 5)

D = zeros(size(C,1), size(B,2));    % no direct feedthrough

%% 6) Discretisation (ZOH) 
% ZOH matches the digital controller behaviour: u held constant between
% samples.  Result is the exact discrete-time equivalent at sample instants.

sysd = c2d(ss(A, B, C, D), Ts, 'zoh');
Gd   = sysd.A;
Hd   = sysd.B;

%% 7) Controllability and observability 
% Both ranks must equal n for the design to exist (lecture L6, slide 12).

n = size(Gd, 1);   % number of states
p = size(C,  1);   % number of measurements

rank_C = rank(ctrb(Gd, Hd));
rank_O = rank(obsv(Gd, C));

fprintf('\nControllability rank: %d  (need %d)\n', rank_C, n);
fprintf('Observability  rank: %d  (need %d)\n', rank_O, n);
assert(rank_C == n, 'System is not controllable -- design cannot proceed.');
assert(rank_O == n, 'System is not observable  -- observer cannot exist.');

%% 8) LQR state feedback (Bryson's rule) 
% Bryson's rule:
%   Q_ii = 1 / (max acceptable value of state i)^2
%   R_ii = 1 / (max acceptable value of input i)^2

% IMPORTANT -- linearisation-validity-driven tuning.
% A PVTOL is underactuated: the ONLY way it can translate in x is to
% tilt (xddot = -(u2/m) sin theta).  With a large x offset, an
% x-aggressive controller commands a large tilt -- and the whole design
% is linearised at theta = 0 (sin theta ~ theta, cos theta ~ 1), valid
% only for small theta.  So theta must be kept inside the linear region;
% x convergence is deliberately made slower to buy that.

% KEY INSIGHT -- x and y are NOT symmetric for a PVTOL:
%   * y is regulated by modulating total thrust directly
%       yddot = (u2/m) cos theta - g,  cos theta ~ 1 near hover
%     -> regulating y needs NO tilt, so y can stay assertive.
%   * x can ONLY be moved by tilting (xddot = -(u2/m) sin theta)
%     -> an x-aggressive controller commands large theta, leaving the
%        linear region the whole design is built on.
% So the correct tuning is ASYMMETRIC: patient in x to protect theta,
% but still assertive in y because y costs no tilt.

% Tuning targets:
%   state     max acceptable    Q_ii
%     x       2 m               0.25     (was 0.5 m / 4  -> patient: protects theta)
%     xdot    2 m/s             0.25
%     y       0.5 m             4        (UNCHANGED -- y needs no tilt, stay crisp)
%     ydot    2 m/s             0.25
%     theta   0.05 rad          400      (was 0.1 / 100  -> protect tilt)
%     thdot   0.5 rad/s         4        (was 1   / 1    -> damp pitch)
%
%   input     max acceptable    R_ii
%     u1      2 N               1/4
%     u2 dev  5 N               1/25

% Net effect: only the x-channel is slowed (Q_x/Q_theta ratio drops
% ~64x), so peak |theta| is pulled back into the valid linear region
% while y stays fast.  x settling time grows -- that is the deliberate
% trade for staying inside the model.

% R multiplied by 10 (was 5): softer, smaller-effort inputs.

% This is a CONSERVATIVE starting point chosen to be safely inside the
% linear region (likely slow in x).  Use the validity diagnostic added
% to main.m to iterate: if max|theta| is comfortably below the limit,
% raise Q_x (or lower the R multiplier) to speed the x channel up; stop
% just before max|theta| reaches the limit.  That knee is your design.
% (y is already fast and does not affect theta -- leave Q_y alone.)

%             x      xdot    y     ydot   theta  thdot
Q = diag([  0.25,   0.25,   4,    0.25,   400,   4   ]);
R = diag([ 1/2^2,  1/5^2 ]) * 10;

[K, P, e_LQR] = dlqr(Gd, Hd, Q, R);
% K     2x6 feedback gain        ->   u_dev = -K * z_hat
% P     Riccati solution         ->   V(z) = z' P z is Lyapunov function
% e_LQR closed-loop eigenvalues

%% 9) Kalman observer (dual of LQR) 
% Qn  process noise covariance       (n-by-n)
% Rn  measurement noise covariance   (p-by-p)

% Ratio Qn/Rn sets observer aggressiveness:
% higher Qn or lower Rn  ->  trust sensors more  ->  faster observer
% Observer should be MUCH faster than the controller (poles closer to 0)
% so that estimates converge before the controller acts on them.

Qn = diag([0.01,  0.1,  0.01,  0.1,  0.01,  0.1]);
Rn = 0.1 * eye(p);

% The observer in my_controller runs in PREDICTION (Luenberger) form:
% z_hat(k+1) = (Gd - L*C) z_hat(k) + Hd u(k) + L y(k)
% so L must be the prediction-form gain, NOT the dlqe current-estimator
% gain M (where the optimal predictor gain would be Gd*M).

% By the duality eig(Gd - L C) = eig(Gd' - C' L'), the optimal
% prediction-form observer gain is obtained directly as the dual LQR:
%     minimise the dual cost with (A,B) -> (Gd', C') and (Q,R) -> (Qn,Rn)
% This is guaranteed stabilising whenever (Gd, C) is observable
% (asserted in step 7) and matches the recursion used by the controller.

[Kd, Pe] = dlqr(Gd', C', Qn, Rn);
L         = Kd';             % n-by-p prediction-form observer gain
e_obs     = eig(Gd - L*C);   % observer poles = eigenvalues of error dynamics

%% 10) Reference-prefilter / linear-validity constants 
% These tie the reference prefilter (my_controller.m) and the validity
% diagnostic (main.m) to ONE source so they can never disagree.

% The reference command is shaped by a critically-damped 2nd-order
% prefilter (NOT a bang-bang trajectory): this keeps the reference --
% and hence theta, since xddot ~ -g*theta -- SMOOTH (continuous
% acceleration, no corners or chatter), while still bounding the implied
% tilt for validity.

% Sizing: for a critically-damped 2nd-order step of size D the peak
% acceleration is ~ D*wn^2, so the implied tilt is ~ D*wn^2/g.  Choosing
%   wn = sqrt( g * THETA_GOV / DSTEP_MAX )
% keeps |theta| <= THETA_GOV for any commanded step up to DSTEP_MAX,
% with THETA_GOV well inside THETA_VALID for margin.

THETA_VALID = 0.35;     % [rad] hard linear-validity limit (plots/diag)
THETA_GOV   = 0.20;     % [rad] prefilter tilt budget (< THETA_VALID)
DSTEP_MAX   = 5.0;      % [m]   largest commanded setpoint step to protect
WN          = sqrt(g * THETA_GOV / DSTEP_MAX);   % [rad/s] prefilter bandwidth

fprintf('Reference prefilter: critically damped, wn = %.3f rad/s\n', WN);

%% 11) Save and report 

save('designValues.mat', ...
     'K', 'L', 'P', 'Pe', ...
     'Gd', 'Hd', 'C', 'D', ...
     'A', 'B', 'Q', 'R', 'Qn', 'Rn', ...
     'THETA_VALID', 'THETA_GOV', 'DSTEP_MAX', 'WN');

fprintf('\nLQR feedback gain K (2x6):\n');  disp(K);
fprintf('Observer gain L (6x3):\n');         disp(L);

fprintf('Max |closed-loop pole|: %.4f  (need < 1)\n', max(abs(e_LQR)));
fprintf('Max |observer  pole|:  %.4f  (need < 1)\n', max(abs(e_obs)));

assert(max(abs(e_LQR)) < 1, 'Closed-loop unstable -- retune Q/R.');
assert(max(abs(e_obs)) < 1, 'Observer unstable -- retune Qn/Rn.');

fprintf('\nDesign complete.  Results saved to designValues.mat\n');


% (All figures are produced by make_figures.m, kept separate so design.m
%  only DESIGNS and main.m only SIMULATES.  e_LQR and e_obs are also
%  saved so the pole map can be drawn without recomputation.)
save('designValues.mat', 'e_LQR', 'e_obs', '-append');
