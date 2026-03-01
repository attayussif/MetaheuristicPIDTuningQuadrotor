function LQI = build_LQI_controller(p)
% Builds LQI controller around hover (small angles).
% States: [z vz phi p theta q psi r] (8)
% Inputs: [T tauPhi tauTheta tauPsi] (4)
% Outputs for integral: [z phi theta psi] (4)

    % Hover equilibrium
    xeq = [p.z_d; 0; p.roll_d; 0; p.pitch_d; 0; p.yaw_d; 0];
    ueq = [p.m*p.g; 0; 0; 0];

    % Linearized reduced model (small angles, decoupled)
    % zdd ≈ (T/m) - g
    % phidd ≈ tauPhi/Ix, thetadd ≈ tauTheta/Iy, psidd ≈ tauPsi/Iz

    A = zeros(8);
    B = zeros(8,4);

    % z, vz
    A(1,2)=1;
    B(2,1)=1/p.m;

    % phi, p
    A(3,4)=1;
    B(4,2)=1/p.Ix;

    % theta, q
    A(5,6)=1;
    B(6,3)=1/p.Iy;

    % psi, r
    A(7,8)=1;
    B(8,4)=1/p.Iz;

    % Output y = [z phi theta psi]
    C = [1 0 0 0 0 0 0 0;
         0 0 1 0 0 0 0 0;
         0 0 0 0 1 0 0 0;
         0 0 0 0 0 0 1 0];
    D = zeros(4,4);

    % Augment with integral of output error: xI_dot = r - y
    Aaug = [A zeros(8,4);
            -C zeros(4,4)];
    Baug = [B; -D];

    % Choose Q/R (tune these like "weights")
    % Penalize tracking states + integrators strongly, keep control moderate
    Qx = diag([ 40,  8,   40,  8,   40,  8,   20,  4 ]);
    Qi = diag([ 40,  80,  80,  20 ]);
    Qaug = blkdiag(Qx, Qi);

    R  = diag([ 20,  2.0, 2.0, 1.0 ]);

    Kaug = lqr(Aaug, Baug, Qaug, R);

    Kx = Kaug(:,1:8);
    Ki = Kaug(:,9:end);

    LQI.Kx = Kx;
    LQI.Ki = Ki;
    LQI.xeq = xeq;
    LQI.ueq = ueq;

    writematrix(LQI.Kx,'LQI_Kx.csv');
    writematrix(LQI.Ki,'LQI_Ki.csv');
end