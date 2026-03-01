% clc; clear; close all;

% Load best gains
load('best_gains.mat');  % loads best_gains vector

 % initial configuration
    roll_i = -0.1;   % rad
    pitch_i = -0.9;  % rad
    yaw_i = -3;      % rad
    altitude_i = 0;  % meters

 % Desired configuration
    roll_d = -0.1;   % rad
    pitch_d = 0;     % rad
    yaw_d = -pi;     % rad
    altitude_d = 6;  % meters


% Assign gains to base workspace
assignin('base', 'Kp_z',     best_gains(1));
assignin('base', 'Ki_z',     best_gains(2));
assignin('base', 'Kd_z',     best_gains(3));
assignin('base', 'Kp_phi',   best_gains(4));
assignin('base', 'Ki_phi',   best_gains(5));
assignin('base', 'Kd_phi',   best_gains(6));
assignin('base', 'Kp_theta', best_gains(7));
assignin('base', 'Ki_theta', best_gains(8));
assignin('base', 'Kd_theta', best_gains(9));
assignin('base', 'Kp_psi',   best_gains(10));
assignin('base', 'Ki_psi',   best_gains(11));
assignin('base', 'Kd_psi',   best_gains(12));

% Optional: reassign initial and desired states if needed
    assignin('base','roll_i',roll_i);
    assignin('base','pitch_i',pitch_i);
    assignin('base','yaw_i',yaw_i);
    assignin('base','altitude_i',altitude_i);

    assignin('base','roll_d',roll_d);
    assignin('base','pitch_d',pitch_d);
    assignin('base','yaw_d',yaw_d);
    assignin('base','altitude_d',altitude_d);

% Run Simulink model
simOut = sim('quadrotor_model', 'ReturnWorkspaceOutputs','on');
logsout = simOut.logsout;

% Extract signals
z     = logsout.get('z').Values;
phi   = logsout.get('phi').Values;
theta = logsout.get('theta').Values;
psi   = logsout.get('psi').Values;

% Plot altitude
figure;
plot(z.Time, z.Data, 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Altitude z (m)');
title('Altitude Response'); grid on;

% Plot attitude responses
figure;
plot(phi.Time, phi.Data, 'r', theta.Time, theta.Data, 'g', psi.Time, psi.Data, 'b', 'LineWidth', 2);
legend('\phi (Roll)', '\theta (Pitch)', '\psi (Yaw)');
xlabel('Time (s)'); ylabel('Angle (rad)');
title('Attitude Responses'); grid on;
