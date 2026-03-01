function [Q, x, y, z, U, t, Tau, Att] = QuadrotorPerformance(gains)
%QUADROTORPERFORMANCE  MATLAB-only objective (ODE) for quadrotor PID tuning
%
% Returns Q=1e6 only on:
%   - ODE failure
%   - Early-stop event
%   - non-finite Q
%
% Adds one-time debug print to reveal the cause.

    persistent firstFailShown
    if isempty(firstFailShown), firstFailShown = false; end

    p = quad_params();

    % --- Apply gains ---
    p.Kp_z     = gains(1);  p.Ki_z     = gains(2);  p.Kd_z     = gains(3);
    p.Kp_phi   = gains(4);  p.Ki_phi   = gains(5);  p.Kd_phi   = gains(6);
    p.Kp_theta = gains(7);  p.Ki_theta = gains(8);  p.Kd_theta = gains(9);
    p.Kp_psi   = gains(10); p.Ki_psi   = gains(11); p.Kd_psi   = gains(12);

    % --- Initial state ---
    x0 = zeros(16,1);
    x0(5)  = p.z0;
    x0(7)  = p.roll0;
    x0(9)  = p.pitch0;
    x0(11) = p.yaw0;

    % --- ODE options ---
    opts = odeset( ...
        'RelTol', p.RelTol, ...
        'AbsTol', p.AbsTol, ...
        'MaxStep', p.MaxStep, ...
        'Events', @(tt,xx) stopEvent(tt,xx,p));

    % --- Simulate ---
    try
        [t, X, te, ~, ie] = ode15s(@(tt,xx) quad_dynamics(tt,xx,p), [0 p.tf], x0, opts);
    catch ME
        Q = 1e6; x=[]; y=[]; z=[]; U=[]; t=[]; Tau=[]; Att=[];
        if ~firstFailShown
            firstFailShown = true;
            fprintf('\n[QuadrotorPerformance] ODE FAILED -> 1e6\n');
            fprintf('Message: %s\n', ME.message);
        end
        return;
    end

    % --- Event penalty ---
    if ~isempty(te) && any(ie == 1)
        Q = 1e6; x=[]; y=[]; z=[]; U=[]; t=[]; Tau=[]; Att=[];
        if ~firstFailShown
            firstFailShown = true;
            fprintf('\n[QuadrotorPerformance] STOP EVENT TRIGGERED at t=%.4f -> 1e6\n', te(1));
            fprintf('States at stop: z=%.3f, phi=%.3f, theta=%.3f, Vz=%.3f, P=%.3f, Q=%.3f\n', ...
                X(end,5), X(end,7), X(end,9), X(end,6), X(end,8), X(end,10));
        end
        return;
    end

    % --- Extract states ---
    x = X(:,1); y = X(:,3); z = X(:,5);
    phi = X(:,7); theta = X(:,9); psi = X(:,11);
    Att = [phi, theta, psi];

    % --- Tracking cost (vectorized) ---
    ez     = (z     - p.z_d)     / p.z_max;
    ephi   = (phi   - p.roll_d)  / p.angle_max;
    etheta = (theta - p.pitch_d) / p.angle_max;
    epsi   = (psi   - p.yaw_d)   / p.angle_max;

    Q1 = p.W_z*(ez.^2) + p.W_phi*(ephi.^2) + p.W_theta*(etheta.^2) + p.W_psi*(epsi.^2);

    % --- Effort term ALWAYS included ---
    N = numel(t);
    wantU   = (nargout >= 5);
    wantTau = (nargout >= 7);

    if wantU,   U = zeros(N,4); else, U = []; end
    if wantTau, Tau = zeros(N,4); else, Tau = []; end

    Q2 = zeros(N,1);

    for k = 1:N
        [Tcmd, tauPhi, tauTheta, tauPsi, Uk] = quad_pid_control(X(k,:).', p);

        if wantU,   U(k,:)   = Uk; end
        if wantTau, Tau(k,:) = [Tcmd, tauPhi, tauTheta, tauPsi]; end

        Q2(k) = p.W_U * sum((Uk./p.U_norm).^2);
    end

    Q_total = Q1 + p.W_u_scale*Q2;
    Q = trapz(t, Q_total) / max(eps, (t(end) - t(1)));

    if ~isfinite(Q)
        if ~firstFailShown
            firstFailShown = true;
            fprintf('\n[QuadrotorPerformance] Q is non-finite -> 1e6\n');
        end
        Q = 1e6;
    end
end

function [value, isterminal, direction] = stopEvent(~, x, p)
    Z     = x(5);
    Vz    = x(6);
    phi   = x(7);
    P     = x(8);
    theta = x(9);
    Qr    = x(10);

    zBad    = (Z < -2) || (Z > 20);
    angBad  = (abs(phi) > 1.2) || (abs(theta) > 1.2);
    rateBad = (abs(P) > 50) || (abs(Qr) > 50) || (abs(Vz) > 50);

    bad = zBad || angBad || rateBad;

    value = double(~bad);
    isterminal = 1;
    direction  = 0;
end
