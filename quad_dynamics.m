function dx = quad_dynamics(~, x, p)
% x = [X Vx Y Vy Z Vz Phi P Theta Q Psi R Iz Iphi Itheta Ipsi]

    % Unpack
    Xp = x(1);  Vx = x(2);
    Yp = x(3);  Vy = x(4);
    Z  = x(5);  Vz = x(6);

    phi = x(7);   P = x(8);
    th  = x(9);   Q = x(10);
    psi = x(11);  R = x(12);

    iz     = x(13);
    iphi   = x(14);
    itheta = x(15);
    ipsi   = x(16);

    % Compute control + motor mix
    [Tcmd, tauPhi, tauTheta, tauPsi, ~, omega_d] = quad_pid_control(x, p);

    % Rotation (Z-up inertial). Thrust along body +Zb
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(th);  sth  = sin(th);
    cpsi = cos(psi); spsi = sin(psi);

    % R = Rz(psi)*Ry(theta)*Rx(phi)
    Rbi = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
            spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
              -sth,               cth*sphi,               cth*cphi ];

    acc = (1/p.m) * (Rbi * [0;0;Tcmd]) - [0;0;p.g];

    % Translational
    dXp = Vx;
    dVx = acc(1);
    dYp = Vy;
    dVy = acc(2);
    dZ  = Vz;
    dVz = acc(3);

    % Rotational (same structure as your dynamics, with gyro coupling)
    dphi = P;
    dP   = p.a1*Q*R + p.a2*Q*omega_d + tauPhi/p.Ix;

    dth  = Q;
    dQ   = p.a3*P*R + p.a4*P*omega_d + tauTheta/p.Iy;

    dpsi = R;
    dR   = p.a5*P*Q + tauPsi/p.Iz;

    % Integrators (anti-windup via clamp in control function; integrate raw error here)
    e_z   = (p.z_d     - Z);
    e_phi = (p.roll_d  - phi);
    e_th  = (p.pitch_d - th);
    e_psi = (p.yaw_d   - psi);

    diz     = e_z;
    diphi   = e_phi;
    ditheta = e_th;
    dipsi   = e_psi;

    dx = zeros(16,1);
    dx(1)  = dXp;   dx(2)  = dVx;
    dx(3)  = dYp;   dx(4)  = dVy;
    dx(5)  = dZ;    dx(6)  = dVz;
    dx(7)  = dphi;  dx(8)  = dP;
    dx(9)  = dth;   dx(10) = dQ;
    dx(11) = dpsi;  dx(12) = dR;
    dx(13) = diz;   dx(14) = diphi;
    dx(15) = ditheta; dx(16) = dipsi;
end
