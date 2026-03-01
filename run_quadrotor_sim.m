clc;
clear;
close all;

%% ==========================
%  Load parameters
%% ==========================
p = quad_params();

%% ==========================
%  Choose PID gains (manual test)
%  (replace later by GA/DE/PSO)
%% ==========================
gains = [ ...
    0.1069	0.0062	0.4618  ...  % z
    0.0017	0.0001	0.0074  ...  % roll
    0.0017	0.0001	0.0074  ...  % pitch
    0.0030	0.0002	0.0128 ...  % yaw
];


p.Kp_z     = gains(1);  p.Ki_z     = gains(2);  p.Kd_z     = gains(3);
p.Kp_phi   = gains(4);  p.Ki_phi   = gains(5);  p.Kd_phi   = gains(6);
p.Kp_theta = gains(7);  p.Ki_theta = gains(8);  p.Kd_theta = gains(9);
p.Kp_psi   = gains(10); p.Ki_psi   = gains(11); p.Kd_psi   = gains(12);

%% ==========================
%  Initial state
%  [x vx y vy z vz phi p theta q psi r iz iphi itheta ipsi]
%% ==========================
x0 = zeros(16,1);
x0(5)  = p.z0;
x0(7)  = p.roll0;
x0(9)  = p.pitch0;
x0(11) = p.yaw0;

%% ==========================
%  Simulate
%% ==========================
opts = odeset( ...
    'RelTol', p.RelTol, ...
    'AbsTol', p.AbsTol, ...
    'MaxStep', p.MaxStep);

fprintf('Running quadrotor simulation...\n');

[t, X] = ode45(@(t,x) quad_dynamics(t,x,p), [0 p.tf], x0, opts);

fprintf('Simulation finished.\n');

%% ==========================
%  Extract states
%% ==========================
x   = X(:,1);
y   = X(:,3);
z   = X(:,5);

phi   = X(:,7);
theta = X(:,9);
psi   = X(:,11);

%% ==========================
%  Recompute control inputs
%% ==========================
N = numel(t);
U = zeros(N,4);

for k = 1:N
    [~,~,~,~,Uk] = quad_pid_control(X(k,:).', p);
    U(k,:) = Uk;
end

%% ==========================
%  Plots (robust)
%% ==========================

set(0,'DefaultFigureWindowStyle','normal');   % avoid dock weirdness

% --- Altitude
fig1 = figure('Name','Altitude Tracking','Color','w','Units','pixels','Position',[80 80 700 420]);
plot(t, z, 'LineWidth', 2); hold on;
yline(p.z_d,'--');
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Tracking');
grid on; axis tight;

% --- Attitude
fig2 = figure('Name','Attitude','Color','w','Units','pixels','Position',[820 80 700 520]);

subplot(3,1,1);
plot(t, phi, 'LineWidth', 1.5); hold on;
yline(p.roll_d,'--');
ylabel('\phi (rad)'); grid on; axis tight;

subplot(3,1,2);
plot(t, theta, 'LineWidth', 1.5); hold on;
yline(p.pitch_d,'--');
ylabel('\theta (rad)'); grid on; axis tight;

subplot(3,1,3);
plot(t, psi, 'LineWidth', 1.5); hold on;
yline(p.yaw_d,'--');
xlabel('Time (s)'); ylabel('\psi (rad)');
grid on; axis tight;

% --- Motor speeds
fig3 = figure('Name','Motor Speeds','Color','w','Units','pixels','Position',[80 540 700 420]);
plot(t, U, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Motor Speeds');
legend('\omega_1','\omega_2','\omega_3','\omega_4','Location','best');
grid on; axis tight;

% --- 3D trajectory
fig4 = figure('Name','3D Trajectory','Color','w','Units','pixels','Position',[820 620 700 420]);
plot3(x, y, z, 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Quadrotor Trajectory');
grid on; axis tight; view(35,22);

drawnow;   % force render
