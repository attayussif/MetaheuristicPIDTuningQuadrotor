% test_quad_stack.m - quick check of QuadrotorPerformance stack

clc; clear; close all;

p = quad_params();

% seed (tuner scale you supplied earlier)
seed = [ ...
    0.1069  0.0062  0.4618  ...  % z
    0.0017  0.0001  0.0074  ...  % roll
    0.0017  0.0001  0.0074  ...  % pitch
    0.0030  0.0002  0.0128  ...  % yaw
];

fprintf('Bounds check ---\n')
disp([min(seed,seed); max(seed,seed)])  % trivial, but shows seed form

% Call objective (request full logs)
[Q, x, y, z, U, t, Tau, Att] = QuadrotorPerformance(seed);

fprintf('Q(seed) = %.6g\n', Q);

if isempty(t)
    error('QuadrotorPerformance returned empty t -> check stop event or error in ODE.');
end

figure; subplot(3,1,1); plot(t,z); hold on; yline(p.z_d,'--'); title('z'); grid on;
subplot(3,1,2); plot(t,Att(:,1)); hold on; yline(p.roll_d,'--'); title('phi'); grid on;
subplot(3,1,3); plot(t, U); title('omega (motor speeds)'); grid on;

% show first/last few Tau
disp('Tau first 5 rows:'); disp(Tau(1:min(5,size(Tau,1)),:));
