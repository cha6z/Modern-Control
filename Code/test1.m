%% prefilter_bandwidth_sensitivity.m
% Purpose:
%   Sweep reference prefilter bandwidth and show the trade-off between
%   tracking speed and pitch validity.

clear; clc;

SystemParameters
S0 = load('designValues.mat');

WN_scales = [0.5 0.75 1.0 1.25 1.5 2.0];
t_span = [0 30];

results = [];

fprintf('\n=== Reference prefilter bandwidth sensitivity ===\n');
fprintf('%8s | %10s | %12s | %10s | %12s\n', ...
    'WN scale', 'WN rad/s', 'max|theta|', 'final err', 'valid?');
fprintf('%s\n', repmat('-', 1, 68));

for i = 1:numel(WN_scales)
    alpha = WN_scales(i);

    S = S0;
    S.WN = alpha*S0.WN;
    save('designValues_temp.mat', '-struct', 'S');

    global OBS_LOG
    OBS_LOG = [];

    [T, Y, U] = System_Simulator(@VTOL_Dynamics_corrected, x0, t_span, ...
                                 @(t,y) my_controller_temp(t,y), Ts, studentID);

    ref_state = zeros(numel(T), 6);
    for k = 1:numel(T)
        rc = reference_signal(T(k));
        ref_state(k,1) = rc(1);
        ref_state(k,3) = rc(2);
    end

    final_err = norm([Y(end,1)-ref_state(end,1), Y(end,3)-ref_state(end,3)]);
    max_theta = max(abs(Y(:,5)));
    is_valid  = max_theta < S0.THETA_VALID;

    results = [results; alpha, S.WN, max_theta, final_err, is_valid];

    fprintf('%8.2f | %10.4f | %12.4f | %10.4f | %12d\n', ...
        alpha, S.WN, max_theta, final_err, is_valid);
end

delete('designValues_temp.mat');

T_results = array2table(results, ...
    'VariableNames', {'WN_scale','WN_rad_s','max_abs_theta_rad', ...
                      'final_error_m','valid'});
writetable(T_results, fullfile('PVTOL_Figures','prefilter_bandwidth_sensitivity.csv'));

fprintf('\nSaved: PVTOL_Figures/prefilter_bandwidth_sensitivity.csv\n');