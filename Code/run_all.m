%% run_all.m -- Top-level entry point for the VTOL control demo

% Runs the complete pipeline (clean separation of concerns):
%   1) design.m         - DESIGN only: LQR + observer + save gains
%   2) main.m           - SIMULATE only + print numeric diagnostics,
%                          then calls:
%        make_figures.m  - ALL plots (states, inputs, pole map, observer)
%        Animate_VTOL.m  - animation
%   reference_signal.m   - single source of the commanded setpoint
%
% Use this single command in the demo to ensure everything runs in the
% correct order with no missing state from previous runs.

clear; close all; clc;


fprintf('  EGH445 VTOL Control Demo -- Run All\n');


fprintf('>> Step 1/2: Designing controller and observer ...\n');
design

fprintf('\n');
fprintf('>> Step 2/2: Running closed-loop simulation ...\n');
main

fprintf('\nDone.\n');
