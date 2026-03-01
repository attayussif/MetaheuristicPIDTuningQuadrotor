function [omega, omega_d] = motor_allocation(u, b, d)
% MOTOR_ALLOCATION
% Inputs:
%   u = [T; tau_phi; tau_theta; tau_psi]
%   b = thrust coefficient
%   d = drag coefficient
%
% Outputs:
%   omega   = [ω1 ω2 ω3 ω4]  (rad/s, >= 0)
%   omega_d = -ω1 + ω2 - ω3 + ω4

    T       = u(1);
    tau_phi = u(2);
    tau_th  = u(3);
    tau_psi = u(4);

    % Inversion logic (your original algebra)
    A = (T - tau_phi - tau_th) / (2*b);
    B = (tau_psi - d*(tau_phi - tau_th)/b) / (2*d);

    w2 = (A + B) / 2;
    w1 = (A - B) / 2;

    w3 = (tau_th / b) + w1;
    w4 = (tau_phi / b) + w2;

    % Enforce non-negative squared speeds
    w1 = max(w1, 0);
    w2 = max(w2, 0);
    w3 = max(w3, 0);
    w4 = max(w4, 0);

    % Rotor speeds
    omega = sqrt([w1 w2 w3 w4]);

    % Rotor imbalance for gyroscopic coupling
    omega_d = -omega(1) + omega(2) - omega(3) + omega(4);
end
