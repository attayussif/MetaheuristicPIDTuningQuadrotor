function u_act = motor_forward(omega, b, d, l)
% MOTOR_FORWARD  Compute T and body torques from rotor speeds
%  omega: 1x4 rotor speeds [w1 w2 w3 w4] (rad/s)
%  b,d,l: thrust coeff, drag coeff, arm length
%
% Returns:
%  u_act = [T; tau_phi; tau_theta; tau_psi]

    if nargin < 4
        % keep backward compatibility (some calls pass p.l separately)
        l = 0.225;
    end

    % squared speeds -> thrust per rotor
    w2 = (omega(:).^2).';   % ensure row [1x4]
    fi = b * w2;            % thrust per rotor

    % Total thrust
    T = sum(fi);

    % Roll torque: difference between right/left pairs (convention)
    % Here mapping matches your earlier inverse algebra choices:
    % tau_phi  = l * (f4 - f2)
    tau_phi = l * (fi(4) - fi(2));

    % Pitch torque: difference between front/back pairs
    % tau_theta = l * (f3 - f1)
    tau_theta = l * (fi(3) - fi(1));

    % Yaw torque: drag torque based on rotor spinning direction with sign pattern
    % convection: tau_psi = d * (-w1^2 + w2^2 - w3^2 + w4^2)
    tau_psi = d * (-w2(1) + w2(2) - w2(3) + w2(4));

    u_act = [T; tau_phi; tau_theta; tau_psi];
end
