
function save_metrics_csv(prefix, t, z, phi, theta, psi, Tau, U, Q, p)
% save_metrics_csv
% Saves time histories and summary metrics to CSV for paper use.
%
% prefix : string, e.g. 'GA', 'PID', 'DE'
% t      : Nx1 time vector
% z      : Nx1 altitude
% phi,theta,psi : Nx1 attitudes
% Tau    : Nx4 [Tcmd tau_phi tau_theta tau_psi]
% U      : Nx4 motor speeds
% Q      : scalar fitness
% p      : quad_params struct

    %% ==========================
    %  Time histories
    %% ==========================
    data = table( ...
        t(:), ...
        z(:), ...
        phi(:), ...
        theta(:), ...
        psi(:), ...
        Tau(:,1), Tau(:,2), Tau(:,3), Tau(:,4), ...
        U(:,1), U(:,2), U(:,3), U(:,4), ...
        'VariableNames', { ...
            'time_s', ...
            'z_m', ...
            'phi_rad', 'theta_rad', 'psi_rad', ...
            'Tcmd_N', 'tau_phi_Nm', 'tau_theta_Nm', 'tau_psi_Nm', ...
            'omega1', 'omega2', 'omega3', 'omega4'});

    writetable(data, sprintf('%s_time_history.csv', prefix));

    %% ==========================
    %  Time-domain metrics
    %% ==========================
    z_d = p.z_d;
    z_err = z - z_d;

    % Rise time (10%–90%)
    z10 = 0.1*z_d;
    z90 = 0.9*z_d;
    idx10 = find(z >= z10, 1, 'first');
    idx90 = find(z >= z90, 1, 'first');
    rise_time = NaN;
    if ~isempty(idx10) && ~isempty(idx90)
        rise_time = t(idx90) - t(idx10);
    end

    % Settling time (±2%)
    tol = 0.02*z_d;
    settle_idx = find(abs(z_err) > tol, 1, 'last');
    settling_time = NaN;
    if ~isempty(settle_idx)
        settling_time = t(settle_idx);
    end

    % Overshoot
    overshoot = (max(z) - z_d)/z_d * 100;

    % Steady-state error
    ss_error = mean(abs(z_err(end-round(0.1*numel(z)):end)));

    % Control effort
    effort_tau = sum(Tau(:,2:4).^2, 2);
    mean_tau_effort = trapz(t, effort_tau) / (t(end)-t(1));

    max_T = max(Tau(:,1));
    max_tau = max(abs(Tau(:,2:4)), [], 'all');

    metrics = table( ...
        Q, rise_time, settling_time, overshoot, ss_error, ...
        mean_tau_effort, max_T, max_tau, ...
        'VariableNames', { ...
            'fitness_Q', ...
            'rise_time_s', ...
            'settling_time_s', ...
            'overshoot_percent', ...
            'steady_state_error_m', ...
            'mean_torque_effort', ...
            'max_thrust_N', ...
            'max_torque_Nm'});

    writetable(metrics, sprintf('%s_metrics.csv', prefix));
end
