%% SystemParameters.m -- Physical and simulation constants for the PVTOL
%
% Single source of truth for all physical and simulation parameters.
% This is a script -- every other file invokes it via "SystemParameters"
% to pull the constants into its workspace.
%
% Editing any value here propagates throughout the project, provided every
% file that uses parameters calls this script (rather than hardcoding).
%
% Units follow SI throughout.
% =========================================================================

% --- Physical parameters --------------------------------------------------
m = 4;        % aircraft mass                          [kg]
J = 0.0475;   % moment of inertia about centre of mass [kg.m^2]
r = 0.25;     % thrust offset (torque arm length)      [m]
g = 9.81;     % gravitational acceleration             [m/s^2]
c = 0.05;     % translational damping coefficient      [N.s/m]

% --- Simulation parameters ------------------------------------------------
Ts = 0.01;            % discrete sample time           [s]    -> 100 Hz
studentID = 10695141; % unique simulator seed

% --- Initial condition ----------------------------------------------------
% State order: [x; xdot; y; ydot; theta; thetadot]
% Defined HERE (single source) so main.m and make_figures.m use the
% SAME starting point -- change it once and every simulation/plot
% follows.  Aircraft starts 10 m across, 2 m up, tilted 0.2 rad.
x0 = [10; 0; 2; 0; 0.2; 0];
