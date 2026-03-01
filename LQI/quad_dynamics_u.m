function dx = quad_dynamics_u(~, x, u_cmd, p)
% quad_dynamics_u
% State x = [X Vx Y Vy Z Vz Phi P Theta Q Psi R] (12)
% Input  u_cmd = [T; tau_phi; tau_theta; tau_psi]
% Actuators are applied inside via apply_actuators()

    % Unpack
    Xp = x(1);  Vx = x(2);
    Yp = x(3);  Vy = x(4);
    Z  = x(5);  Vz = x(6);

    phi = x(7);  P = x(8);
    th  = x(9);  Q = x(10);
    psi = x(11); R = x(12);

    % Apply actuators
    [u_act, ~, omega_d] = apply_actuators(u_cmd, p);
    Tcmd     = u_act(1);
    tauPhi   = u_act(2);
    tauTheta = u_act(3);
    tauPsi   = u_act(4);

    % Rotation matrix (body->inertial)
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(th);  sth  = sin(th);
    cpsi = cos(psi); spsi = sin(psi);

    Rbi = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
            spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
              -sth,               cth*sphi,               cth*cphi ];

    acc = (1/p.m) * (Rbi * [0;0;Tcmd]) - [0;0;p.g];

    % Translational
    dXp = Vx;   dVx = acc(1);
    dYp = Vy;   dVy = acc(2);
    dZ  = Vz;   dVz = acc(3);

    % Rotational (with gyro coupling consistent with your PID model)
    dphi = P;
    dP   = p.a1*Q*R + p.a2*Q*omega_d + tauPhi/p.Ix;

    dth  = Q;
    dQ   = p.a3*P*R + p.a4*P*omega_d + tauTheta/p.Iy;

    dpsi = R;
    dR   = p.a5*P*Q + tauPsi/p.Iz;

    dx = zeros(12,1);
    dx(1)=dXp; dx(2)=dVx;
    dx(3)=dYp; dx(4)=dVy;
    dx(5)=dZ;  dx(6)=dVz;
    dx(7)=dphi;dx(8)=dP;
    dx(9)=dth; dx(10)=dQ;
    dx(11)=dpsi;dx(12)=dR;
end