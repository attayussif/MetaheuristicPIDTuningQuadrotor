clc; clear; close all;

p = quad_params();

% Build LQI controller
LQI = build_LQI_controller(p);

% Save controller for reuse
save('LQI_best_gains.mat','LQI');

% Simulate once
[Q,x,y,z,U,t,Tau,Att] = LQI_QuadrotorPerformance(LQI);
phi   = Att(:,1);
theta = Att(:,2);
psi   = Att(:,3);

fprintf('LQI: Q=%.4f | omega_max=%.2f rad/s | T_max=%.2f N | tau_max=%.3f Nm\n', ...
    Q, max(U,[],'all'), max(Tau(:,1)), max(abs(Tau(:,2:4)),[],'all'));

save_metrics_csv('LQI', t, z, phi, theta, psi, Tau, U, Q, p);