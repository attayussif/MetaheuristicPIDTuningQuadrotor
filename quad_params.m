function p = quad_params()
%QUAD_PARAMS  Centralized parameter set for MATLAB quadrotor simulation
%
% Keep this file as the single source of truth for parameter names used
% across QuadrotorPerformance, quad_pid_control, motor_allocation, etc.

    % ----------------------------
    % Physical parameters
    % ----------------------------
    p.g = 9.81;
    p.m = 0.468;

    % Thrust/drag coefficients (from your script)
    p.b = 313e-6;
    p.d = 7.5e-7;

    % Arm length
    p.l = 0.225;

    % Inertias
    p.Ix = 75e-4;
    p.Iy = 75e-4;
    p.Iz = 13e-3;

    % Rotor inertia (gyro coupling)
    p.Jr = 6e-5;

    % Gyroscopic coupling coefficients
    p.a1 = (p.Iy - p.Iz)/p.Ix;
    p.a2 = p.Jr / p.Ix;
    p.a3 = (p.Iz - p.Ix)/p.Iy;
    p.a4 = p.Jr / p.Iy;
    p.a5 = (p.Ix - p.Iy)/p.Iz;

    % ----------------------------
    % Desired/initial configuration
    % ----------------------------
    p.roll_d  = -0.1;
    p.pitch_d =  0.0;
    p.yaw_d   = -pi;
    p.z_d     =  6.0;

    p.roll0  = -0.1;
    p.pitch0 = -0.9;
    p.yaw0   = -3.0;
    p.z0     =  0.0;

    % ----------------------------
    % Simulation settings
    % ----------------------------
    p.tf = 15;          % seconds
    p.RelTol = 1e-5;
    p.AbsTol = 1e-7;
    p.MaxStep = 0.02;

    % ----------------------------
    % Integral windup clamps
    % ----------------------------
    p.iLim_z     = 5;
    p.iLim_phi   = 2;
    p.iLim_theta = 2;
    p.iLim_psi   = 2;

    % % ----------------------------
    % % Actuator limits / safety caps
    % % ----------------------------
    % % Choose a physically meaningful rotor speed limit:
    % p.omega_max = 1200;            % rad/s (as you used earlier)
    % % Cost normalization (legacy: keep 11 for comparability with previous cost)
    % p.U_norm = p.omega_max;                 % used to normalize control effort in cost
    % 
    % % Max thrust based on omega_max (physics-consistent)
    % p.T_max = 40;
    % 
    % % Torque cap: conservative physical bound (order-of-magnitude)
    % p.tau_max = p.l * (p.T_max/2);

    p.T_max     = 40;                            % N ( requirement)
    p.omega_max = sqrt(p.T_max/(4*p.b));         % rad/s
    p.tau_max   = p.l * (p.T_max/2);             % conservative Nm
    
    % effort normalization MUST match what U actually is (omega)
    p.U_norm = p.omega_max;

    % ----------------------------
    % Cost weights (keep editable)
    % ----------------------------
    p.z_max = 11;
    p.angle_max = pi/2;

    % Tracking weights
    p.W_z     = 15;
    p.W_phi   = 10;
    p.W_theta = 10;
    p.W_psi   = 10;

    % Control-effort weight and scaling (used in QuadrotorPerformance)
    p.W_U = 8;           % original you used 8 for GA run; adjust as needed
    p.W_u_scale = 5e-5;  % multiply Q2 before adding to Q1

    % Small safety: maximum allowed control effort normalization
    % (keeps Q2 magnitude reasonable)
end
