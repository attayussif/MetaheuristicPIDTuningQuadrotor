function [Q, x, y, z, U, t, Tau, Att] = QuadrotorPerformance(gains)
% MATLAB-only replacement for Simulink-based QuadrotorPerformance
% Outputs:
%   Q   : scalar cost
%   x,y,z : position trajectories
%   U   : rotor speeds (N×4)
%   t   : time vector
%   Tau : [Tcmd tauPhi tauTheta tauPsi] (N×4)
%   Att : [phi theta psi] (N×3)

    p = quad_params();

    % ----------------------------
    % PID gains (12)
    % ----------------------------
    p.Kp_z     = gains(1);  p.Ki_z     = gains(2);  p.Kd_z     = gains(3);
    p.Kp_phi   = gains(4);  p.Ki_phi   = gains(5);  p.Kd_phi   = gains(6);
    p.Kp_theta = gains(7);  p.Ki_theta = gains(8);  p.Kd_theta = gains(9);
    p.Kp_psi   = gains(10); p.Ki_psi   = gains(11); p.Kd_psi   = gains(12);

    % ----------------------------
    % Initial state
    % [x vx y vy z vz phi p theta q psi r iz iphi itheta ipsi]
    % ----------------------------
    x0 = zeros(16,1);
    x0(1)  = 0;       x0(2)  = 0;
    x0(3)  = 0;       x0(4)  = 0;
    x0(5)  = p.z0;    x0(6)  = 0;
    x0(7)  = p.roll0; x0(8)  = 0;
    x0(9)  = p.pitch0;x0(10) = 0;
    x0(11) = p.yaw0;  x0(12) = 0;
    % integrators (13..16) are zero

    opts = odeset('RelTol',p.RelTol,'AbsTol',p.AbsTol,'MaxStep',p.MaxStep);

    try
        [t, X] = ode45(@(tt,xx) quad_dynamics(tt,xx,p), [0 p.tf], x0, opts);
    catch
        Q = 1e6; x=[]; y=[]; z=[]; U=[]; t=[]; Tau=[]; Att=[];
        return;
    end

    % ----------------------------
    % Extract trajectories
    % ----------------------------
    x = X(:,1);
    y = X(:,3);
    z = X(:,5);

    phi   = X(:,7);
    theta = X(:,9);
    psi   = X(:,11);

    Att = [phi, theta, psi];

    % ----------------------------
    % Recompute control history + cost
    % ----------------------------
    N = numel(t);
    U   = zeros(N,4);
    Tau = zeros(N,4);
    Q_total = zeros(N,1);

    for k = 1:N
        [Tcmd, tauPhi, tauTheta, tauPsi, Uk] = quad_pid_control(X(k,:).', p);

        U(k,:)   = Uk;
        Tau(k,:) = [Tcmd, tauPhi, tauTheta, tauPsi];

        ez     = (z(k)     - p.z_d)     / p.z_max;
        ephi   = (phi(k)   - p.roll_d)  / p.angle_max;
        etheta = (theta(k) - p.pitch_d) / p.angle_max;
        epsi   = (psi(k)   - p.yaw_d)   / p.angle_max;

        Q1 = p.W_z*ez^2 + p.W_phi*ephi^2 + p.W_theta*etheta^2 + p.W_psi*epsi^2;

        % rotor-speed effort term (normalized)
        Q2 = p.W_U * sum((Uk./p.U_norm).^2);

        Q_total(k) = Q1 + p.W_u_scale*Q2;
    end

    % Time-average cost
    Q = trapz(t, Q_total) / max(eps, (t(end)-t(1)));

    if ~isfinite(Q)
        Q = 1e6;
    end
end
