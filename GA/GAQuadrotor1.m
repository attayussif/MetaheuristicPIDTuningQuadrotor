clc;
clear;
close all;

%% === Problem Setup ===

% PID tuning bounds for [Kp_z Ki_z Kd_z ... Kd_psi]
lb = [ 
    0.01   1e-5   0.01   ...   % z
    1e-4   1e-6   1e-4   ...   % phi
    1e-4   1e-6   1e-4   ...   % theta
    1e-4   1e-6   1e-4];       % psi

ub = [ 
    5.0    0.2    5.0    ...   % z
    1.0    0.05   1.0    ...   % phi
    1.0    0.05   1.0    ...   % theta
    1.0    0.05   1.0];        % psi

prob = @(g) QuadrotorPerformance(g);   % Objective function (MATLAB ODE baseline)

%% === GA Parameters ===
Np = 15;            % Population size
T = 50;             % Number of generations
Pc = 0.9;           % Crossover probability
Pm = 0.2;           % Mutation probability
elitism_num = 3;    % Elitism count
NRuns = 30;         % Number of GA runs

%% ==========================
%  Timing / runtime tracking
%% ==========================
startTime = datetime('now');
simTimer  = tic;
fprintf('GA simulation started at %s\n', datestr(startTime,'yyyy-mm-dd HH:MM:SS'));

%% === Storage Initialization ===
bestsol     = zeros(NRuns, length(lb));
bestfitness = zeros(NRuns, 1);
BestFitIter = zeros(NRuns, T+1);

% Store trajectories per run (so best-run trajectory is correct)
trajX = cell(NRuns,1);
trajY = cell(NRuns,1);
trajZ = cell(NRuns,1);

% Per-run time
runTime = zeros(NRuns,1);

%% === GA Optimization Loop ===
for i = 1:NRuns
    disp(['GA Run ', num2str(i)]);
    rng(i, 'twister');  % Reproducibility

    % === MATLAB baseline cleanup (optional but helps if objective uses persistent vars) ===
    % NOTE: This can slow things down. Keep only if needed.
    % clear functions;

    % Per-run timer
    tRun = tic;

    % NOTE:
    % GAbasic keeps x,y,z empty to avoid extra simulations inside the GA loop
    [bestsol(i,:), bestfitness(i), BestFitIter(i,:), ~, ~, ~, ~, ~] = ...
        GAbasic(prob, lb, ub, Np, T, Pc, Pm, elitism_num);

    runTime(i) = toc(tRun);

    % Save trajectory for this run (ONE simulation per run, stable)
    % This requires QuadrotorPerformance to return [Q, x, y, z] (or more)
    [~, x, y, z] = QuadrotorPerformance(bestsol(i,:));
    trajX{i} = x;
    trajY{i} = y;
    trajZ{i} = z;

    fprintf('Run %d/%d finished | best fitness = %.6g | time = %s\n', ...
            i, NRuns, bestfitness(i), char(seconds(runTime(i))));

    % Simple ETA (based on mean so far)
    avgRun = mean(runTime(1:i));
    etaSec = avgRun * (NRuns - i);
    fprintf('Estimated remaining time: %s\n', char(seconds(etaSec)));
end

% Save final best solution (correct: choose best run, not last run)
[globalBestFitness, bestRunIndex] = min(bestfitness);
best_gains = bestsol(bestRunIndex,:);

save('best_gains.mat', 'best_gains', 'bestRunIndex', 'globalBestFitness');

% Save run results (explicit)
save('Ga_results.mat');

%% === Summary Statistics ===
Stat(1) = min(bestfitness);
Stat(2) = max(bestfitness);
Stat(3) = mean(bestfitness);
Stat(4) = median(bestfitness);
Stat(5) = std(bestfitness);

disp('Fitness Statistics: [min max mean median std]');
disp(Stat);

fprintf('Runtime per run (mean ± std): %.2f ± %.2f seconds\n', mean(runTime), std(runTime));

%% ==========================
%  End-of-simulation timing
%% ==========================
elapsedSec = toc(simTimer);
endTime = datetime('now');

fprintf('\nGA simulation completed at %s\n', datestr(endTime,'yyyy-mm-dd HH:MM:SS'));
fprintf('Total runtime: %s\n', char(seconds(elapsedSec)));

%% ==========================
%  GA Results – Plots
%% ==========================

% Define plot helpers (these were missing in your script)
iters   = 0:T;
meanFit = mean(BestFitIter, 1);
stdFit  = std(BestFitIter, 0, 1);

%% === Plot 1: Mean Convergence Curve ===
figure('Color','w');
plot(iters, meanFit, 'b', 'LineWidth', 2);
xlabel('Generation');
ylabel('Average Best Fitness');
title('Mean Convergence (GA)');
grid on;

%% === Plot 2: Mean ± Std Shaded Area ===
figure('Color','w');
fill([iters, fliplr(iters)], ...
     [meanFit + stdFit, fliplr(meanFit - stdFit)], ...
     [0.85 0.9 1], 'EdgeColor', 'none');
hold on;
plot(iters, meanFit, 'b', 'LineWidth', 2);
xlabel('Generation');
ylabel('Fitness');
title('Mean ± Std of Convergence (GA)');
grid on;

%% === Plot 3: Boxplot of Final Fitness ===
figure('Color','w');
boxplot(bestfitness, 'Notch', 'on');
ylabel('Final Best Fitness');
title('Distribution of Final Fitness over Runs (GA)');
grid on;

%% === Plot 4: 3D Trajectory of Best Run ===
best_x = trajX{bestRunIndex};
best_y = trajY{bestRunIndex};
best_z = trajZ{bestRunIndex};

figure('Color','w');
plot3(best_x, best_y, best_z, 'b', 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory of Best Run (GA)');
axis tight;
view(35,22);

%% ==========================
%  Best GA evaluation (ONE run)
%  Control Effort + Attitude
%% ==========================

p = quad_params();   % for limits

% Use the updated QuadrotorPerformance that returns control + attitude
% Expected signature:
% [Q, x, y, z, U, t, Tau, Att] = QuadrotorPerformance(gains)
[Qbest, xB, yB, zB, UB, tB, TauB, AttB] = QuadrotorPerformance(best_gains);

Tcmd     = TauB(:,1);
tauPhi   = TauB(:,2);
tauTheta = TauB(:,3);
tauPsi   = TauB(:,4);

phiB   = AttB(:,1);
thetaB = AttB(:,2);
psiB   = AttB(:,3);

%%Simulation Metrics
save_metrics_csv('GA', tB, zB, phiB, thetaB, psiB, TauB, UB, Qbest, p);

GA_summary = table( ...
    min(bestfitness), max(bestfitness), mean(bestfitness), median(bestfitness), std(bestfitness), ...
    mean(runTime), std(runTime), ...
    'VariableNames', {'Q_min','Q_max','Q_mean','Q_median','Q_std','runtime_mean_s','runtime_std_s'} );

writetable(GA_summary,'GA_summary_stats.csv');
%% ==========================
%  Control Effort – Best GA
%% ==========================

%% --- Thrust ---
figure('Color','w');
plot(tB, Tcmd, 'LineWidth', 2);
hold on;
yline(p.T_max,'--');
yline(0,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('T_{cmd} (N)');
title('Best GA Solution: Thrust Command');

%% --- Roll Torque ---
figure('Color','w');
plot(tB, tauPhi, 'LineWidth', 2);
hold on;
yline(p.tau_max,'--');
yline(-p.tau_max,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\tau_{\phi} (N·m)');
title('Best GA Solution: Roll Torque');

%% --- Pitch Torque ---
figure('Color','w');
plot(tB, tauTheta, 'LineWidth', 2);
hold on;
yline(p.tau_max,'--');
yline(-p.tau_max,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\tau_{\theta} (N·m)');
title('Best GA Solution: Pitch Torque');

%% --- Yaw Torque ---
figure('Color','w');
plot(tB, tauPsi, 'LineWidth', 2);
hold on;
yline(p.tau_max,'--');
yline(-p.tau_max,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\tau_{\psi} (N·m)');
title('Best GA Solution: Yaw Torque');

%% ==========================
%  Attitude Tracking (Best GA)
%% ==========================

% --- Roll (phi) ---
figure('Color','w');
plot(tB, phiB, 'LineWidth', 2); hold on;
yline(p.roll_d,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\phi (rad)');
title('Best GA Solution: Roll Angle Tracking');

% --- Pitch (theta) ---
figure('Color','w');
plot(tB, thetaB, 'LineWidth', 2); hold on;
yline(p.pitch_d,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Best GA Solution: Pitch Angle Tracking');

% --- Yaw (psi) ---
figure('Color','w');
plot(tB, psiB, 'LineWidth', 2); hold on;
yline(p.yaw_d,'--');
grid on; axis tight;
xlabel('Time (s)');
ylabel('\psi (rad)');
title('Best GA Solution: Yaw Angle Tracking');


figure('Color','w');
plot(tB, zB, 'LineWidth', 2); hold on;
yline(p.z_d,'--','LineWidth',1.5);
grid on; axis tight;
xlabel('Time (s)');
ylabel('z (m)');
title('Best GA Solution: Altitude Tracking');
