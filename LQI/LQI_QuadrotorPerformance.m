function [Q, x, y, z, U, t, Tau, Att] = LQI_QuadrotorPerformance(LQI)
% LQI_QuadrotorPerformance
% Nonlinear sim with LQI, same output format as QuadrotorPerformance.

    p = quad_params();

    % ---------- initial plant state (12) ----------
    x0 = zeros(12,1);
    x0(5)  = p.z0;
    x0(7)  = p.roll0;
    x0(9)  = p.pitch0;
    x0(11) = p.yaw0;

    % ---------- integral state for [z,phi,theta,psi] ----------
    xI0 = zeros(4,1);

    X0 = [x0; xI0];

    opts = odeset('RelTol',p.RelTol,'AbsTol',p.AbsTol,'MaxStep',p.MaxStep);

    try
        [t, X] = ode45(@(tt,XX) lqi_aug_dynamics(tt,XX,p,LQI), [0 p.tf], X0, opts);
    catch
        Q = 1e6; x=[]; y=[]; z=[]; U=[]; t=[]; Tau=[]; Att=[];
        return;
    end

    plant = X(:,1:12);
    x  = plant(:,1);
    y  = plant(:,3);
    z  = plant(:,5);

    phi   = plant(:,7);
    theta = plant(:,9);
    psi   = plant(:,11);
    Att = [phi theta psi];

    % Recompute control + cost (to match your metrics pipeline)
    N = numel(t);
    U   = zeros(N,4);   % omega
    Tau = zeros(N,4);   % [Tcmd tauPhi tauTheta tauPsi]
    Q_total = zeros(N,1);

    for k = 1:N
        plantk = plant(k,:).';
        xIk    = X(k,13:16).';

        [u_cmd] = lqi_control_law(plantk, xIk, p, LQI);

        [u_act, omega, ~] = apply_actuators(u_cmd, p);

        Tau(k,:) = u_act(:).';
        U(k,:)   = omega(:).';

        % Tracking cost (same structure)
        ez     = (z(k)     - p.z_d)     / p.z_max;
        ephi   = (phi(k)   - p.roll_d)  / p.angle_max;
        etheta = (theta(k) - p.pitch_d) / p.angle_max;
        epsi   = (psi(k)   - p.yaw_d)   / p.angle_max;

        Q1 = p.W_z*ez^2 + p.W_phi*ephi^2 + p.W_theta*etheta^2 + p.W_psi*epsi^2;

        % IMPORTANT: normalize rotor speed with omega_max (not 11)
        Q2 = p.W_U * sum((omega./p.U_norm).^2);

        Q_total(k) = Q1 + p.W_u_scale*Q2;
    end

    Q = trapz(t, Q_total) / max(eps, (t(end)-t(1)));
    if ~isfinite(Q), Q = 1e6; end
end

% ===== augmented dynamics: plant + integrators =====
function dX = lqi_aug_dynamics(~, X, p, LQI)
    plant = X(1:12);
    xI    = X(13:16);

    u_cmd = lqi_control_law(plant, xI, p, LQI);

    dplant = quad_dynamics_u(0, plant, u_cmd, p);

    % integral: xI_dot = r - y, for y=[z phi theta psi]
    z   = plant(5);
    phi = plant(7);
    th  = plant(9);
    psi = plant(11);

    r = [p.z_d; p.roll_d; p.pitch_d; p.yaw_d];
    y = [z; phi; th; psi];

    dxI = r - y;

    dX = [dplant; dxI];
end

% ===== LQI control law (reduced state) =====
function u_cmd = lqi_control_law(plant, xI, p, LQI)
    % Reduced state: [z vz phi p theta q psi r]
    z   = plant(5);  vz  = plant(6);
    phi = plant(7);  P   = plant(8);
    th  = plant(9);  Qr  = plant(10);
    psi = plant(11); R   = plant(12);

    xr = [z; vz; phi; P; th; Qr; psi; R];

    dx = xr - LQI.xeq;
    u_cmd = LQI.ueq - LQI.Kx*dx - LQI.Ki*xI;
end