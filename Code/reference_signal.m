function r = reference_signal(t)
% REFERENCE_SIGNAL  Commanded hover setpoint [x_ref; y_ref] (m) vs time.
%
% TRACKING IS NOW ACTIVE (was regulation-only before).
%
% Schedule for the 30 s run:
%   0 - 15 s : command = (0, 0)  -> the aircraft recovers from its
%              initial offset x0 = (10, 2) and settles at the origin.
%              This is the pure REGULATION behaviour.
%   15 s+    : command steps to (4, 2)  -> the controller now TRACKS a
%              new non-zero setpoint.  This is the REFERENCE TRACKING
%              behaviour.
%
% The two phases are deliberately separated in time so the demo shows
% regulation and tracking as distinct, unambiguous events (rather than
% the tracking response being tangled up with the initial-condition
% transient).  The (4, 2) move is within the prefilter's validity
% budget (DSTEP_MAX = 5 m in design.m), so the smooth critically-damped
% prefilter in my_controller keeps the implied tilt theta inside the
% linear-validity region throughout.  No other file needs changing.

    r = [0; 0];                       % phase 1: regulate to origin

    % ---- Reference tracking command (ACTIVE) -------------------------
    if t >= 15
        r = [4; 2];                   % phase 2: track new setpoint
    end                               % (smooth, validity-safe step)
    % ------------------------------------------------------------------

    r = r(:);                         % guarantee column
end
