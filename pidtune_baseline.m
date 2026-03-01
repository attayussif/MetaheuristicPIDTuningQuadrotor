function gains = pidtune_baseline()
% Baseline PID gains using MATLAB pidtune (no Simulink license)

p = quad_params();

s = tf('s');

%% === Altitude ===
Gz = 1/(p.m*s^2);
Cz = pidtune(Gz,'PID');
Kp_z = Cz.Kp;
Ki_z = Cz.Ki;
Kd_z = Cz.Kd;

%% === Roll ===
Gphi = 1/(p.Ix*s^2);
Cphi = pidtune(Gphi,'PID');
Kp_phi = Cphi.Kp;
Ki_phi = Cphi.Ki;
Kd_phi = Cphi.Kd;

%% === Pitch ===
Gtheta = 1/(p.Iy*s^2);
Ctheta = pidtune(Gtheta,'PID');
Kp_theta = Ctheta.Kp;
Ki_theta = Ctheta.Ki;
Kd_theta = Ctheta.Kd;

%% === Yaw ===
Gpsi = 1/(p.Iz*s^2);
Cpsi = pidtune(Gpsi,'PID');
Kp_psi = Cpsi.Kp;
Ki_psi = Cpsi.Ki;
Kd_psi = Cpsi.Kd;

%% === Assemble gain vector ===
gains = [ ...
    Kp_z Ki_z Kd_z ...
    Kp_phi Ki_phi Kd_phi ...
    Kp_theta Ki_theta Kd_theta ...
    Kp_psi Ki_psi Kd_psi ];

save('PIDTUNE_Baseline_Gains.mat','gains');

[Q_pid, xP, yP, zP, UP, tP, TauP, AttP] = QuadrotorPerformance(baseline_gains);


save_ga_results_csv('PID', tP, zP, AttP(:,1), AttP(:,2), AttP(:,3), TauP, UP, Q_pid, quad_params());


end
