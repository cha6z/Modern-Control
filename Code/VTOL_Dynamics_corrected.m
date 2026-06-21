function dx = VTOL_Dynamics_corrected(~, z, u, varargin)
% VTOL_DYNAMICS_CORRECTED  Nonlinear planar VTOL plant dynamics.

% Implements the standard PVTOL model:
%   xddot     = -(u2/m)*sin(theta) - (c/m)*xdot
%   yddot     =  (u2/m)*cos(theta) - g - (c/m)*ydot
%   thetaddot =  (r/J)*u1

% State and input conventions:
%   z = [x; xdot; y; ydot; theta; thetadot]   6x1
%   u = [u1; u2]                              2x1
%       u1 = differential rotor force [N]; produces torque r*u1 about CG
%       u2 = total vertical thrust    [N]; equals m*g at hover

% Signature accepts:
%   VTOL_Dynamics_corrected(t, z, u)              standard call
%   VTOL_Dynamics_corrected(t, z, u, sID, ...)    simulator call (extras ignored)
%
% Parameters are loaded once from SystemParameters.m and cached for speed
% (this function is called many times per second by the integrator).


    %% Load and cache physical parameters 
    persistent m_p J_p r_p g_p c_p
    if isempty(m_p)
        SystemParameters; % populates workspace
        m_p = m;  J_p = J;  r_p = r;  g_p = g;  c_p = c;
    end

    %% Unpack state and input 
    xdot     = z(2);
    ydot     = z(4);
    theta    = z(5);
    thetadot = z(6);
    u1       = u(1);
    u2       = u(2);

    %% Compute state derivative 
    dx     = zeros(6, 1);
    dx(1)  = xdot;                                              % d/dt(x)
    dx(2)  = -(u2/m_p) * sin(theta) - (c_p/m_p) * xdot;         % d/dt(xdot)
    dx(3)  = ydot;                                              % d/dt(y)
    dx(4)  =  (u2/m_p) * cos(theta) - g_p - (c_p/m_p) * ydot;   % d/dt(ydot)
    dx(5)  = thetadot;                                          % d/dt(theta)
    dx(6)  = (r_p/J_p) * u1;                                    % d/dt(thetadot)
end
