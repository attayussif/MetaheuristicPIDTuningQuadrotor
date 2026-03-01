function [Tcmd, tauPhi, tauTheta, tauPsi, U, omega_d] = quad_pid_control(x, p)
% Returns thrust + torques + motor commands (omega^2 proxy) + omega_d for gyro terms
%
% U here is omega (or sqrt(omega^2)) proxy limited by U_max; consistent with your cost usage.

    Z   = x(5);  Vz = x(6);
    phi = x(7);  P  = x(8);
    th  = x(9);  Q  = x(10);
    psi = x(11); R  = x(12);

    iz     = x(13);
    iphi   = x(14);
    itheta = x(15);
    ipsi   = x(16);

    % Errors (desired - actual)
    ez   = p.z_d  - Z;
    ephi = p.roll_d  - phi;
    eth  = p.pitch_d - th;
    epsi = p.yaw_d - psi;

    % Anti-windup clamp (integral state saturation)
    iz     = max(-p.iLim_z,     min(p.iLim_z,     iz));
    iphi   = max(-p.iLim_phi,   min(p.iLim_phi,   iphi));
    itheta = max(-p.iLim_theta, min(p.iLim_theta, itheta));
    ipsi   = max(-p.iLim_psi,   min(p.iLim_psi,   ipsi));

    % --- Outer altitude PID -> vertical accel command ---
    uz = p.Kp_z*ez + p.Ki_z*iz - p.Kd_z*Vz;  % derivative uses -Vz

    % Thrust command (compensate gravity; avoid singularities)
    cphi = cos(phi); cth = cos(th);
    denom = max(0.6, cphi*cth);  % avoid division blow-up
    Tcmd = p.m*(p.g + uz)/denom;

    % --- Attitude PID -> torques ---
    tauPhi   = p.Kp_phi*ephi   + p.Ki_phi*iphi   - p.Kd_phi*P;
    tauTheta = p.Kp_theta*eth  + p.Ki_theta*itheta - p.Kd_theta*Q;
    tauPsi   = p.Kp_psi*epsi   + p.Ki_psi*ipsi   - p.Kd_psi*R;

    % Safety caps (debug stability)
    Tcmd     = max(0, min(p.T_max, Tcmd));
    tauPhi   = max(-p.tau_max, min(p.tau_max, tauPhi));
    tauTheta = max(-p.tau_max, min(p.tau_max, tauTheta));
    tauPsi   = max(-p.tau_max, min(p.tau_max, tauPsi));

    % Assemble control vector
    u = [Tcmd; tauPhi; tauTheta; tauPsi];
    
   % Motor allocation
    [omega, omega_d] = motor_allocation(u, p.b, p.d);
    
    % Saturate PHYSICAL rotor speed
    omega = min(p.omega_max, omega);
    
       % Motor allocation (inverse gives omega), then saturate and recompute actual u:
    [omega, omega_d] = motor_allocation(u, p.b, p.d);
    omega = min(p.omega_max, omega);                  % saturate physically
    u_act = motor_forward(omega, p.b, p.d, p.l);       % recompute actual T/tau
    Tcmd     = u_act(1);
    tauPhi   = u_act(2);
    tauTheta = u_act(3);
    tauPsi   = u_act(4);
    U = omega(:).';

end
