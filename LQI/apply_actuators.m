function [u_act, omega, omega_d] = apply_actuators(u_cmd, p)
% apply_actuators
% u_cmd: [T; tau_phi; tau_theta; tau_psi]
% Returns:
%   u_act  : realized [T; tau_phi; tau_theta; tau_psi]
%   omega  : rotor speeds (rad/s)
%   omega_d: -w1 + w2 - w3 + w4 (gyro coupling term)

    % Command saturation first (safety)
    T       = max(0, min(p.T_max,    u_cmd(1)));
    tau_phi = max(-p.tau_max, min(p.tau_max, u_cmd(2)));
    tau_th  = max(-p.tau_max, min(p.tau_max, u_cmd(3)));
    tau_psi = max(-p.tau_max, min(p.tau_max, u_cmd(4)));

    u_sat = [T; tau_phi; tau_th; tau_psi];

    % Allocation -> omega
    [omega, omega_d] = motor_allocation(u_sat, p.b, p.d);

    % Rotor speed saturation (physical)
    omega = min(p.omega_max, omega);

    % Forward map -> realized u_act (MUST match allocation convention)
    u_act = motor_forward(omega, p.b, p.d, p.l);
end