clc;
clear;
close all;

%% === Problem Setup (match your ODE scale + tuner seed scale) ===
lb = [ ...
    0.01   1e-5   0.01   ...   % z
    1e-4   1e-6   1e-4   ...   % phi
    1e-4   1e-6   1e-4   ...   % theta
    1e-4   1e-6   1e-4];       % psi

ub = [ ...
    10.0   1.0    5.0    ...   % z
    10.0   1.0    5.0    ...   % phi
    10.0   1.0    5.0    ...   % theta
    10.0   1.0    5.0];        % psi

prob = @(g) QuadrotorPerformance(g);   % ODE objective (returns scalar Q when called with 1 output)

%% === TLBO Parameters ===
Np    = 15;      % Population size
T     = 60;      % Iterations (match PSO/GA style if you want)
NRuns = 100;     % Runs (match others for fairness)

%% ==========================
%  Timing / runtime tracking
%% ==========================
startTime = datetime('now');
simTimer  = tic;
fprintf('TLBO simulation started at %s\n', datestr(startTime,'yyyy-mm-dd HH:MM:SS'));

%% === Storage Initialization ===
D = numel(lb);

bestsol     = zeros(NRuns, D);
bestfitness = zeros(NRuns, 1);
BestFitIter = zeros(NRuns, T+1);

trajX = cell(NRuns,1);
trajY = cell(NRuns,1);
trajZ = cell(NRuns,1);

runTime = zeros(NRuns,1);

%% === TLBO Optimization Loop ===
for i = 1:NRuns
    fprintf('\nTLBO Run %d\n', i);
    rng(i, 'twister');

    tRun = tic;

    % TLBObasic should call prob as: f = prob(candidate);
    % (i.e., only need scalar output)
    [bestsol(i,:), bestfitness(i), BestFitIter(i,:), ~, ~] = ...
        TLBObasic(prob, lb, ub, Np, T);

    runTime(i) = toc(tRun);

    % Store one trajectory per run (re-simulate once, stable)
    [~, x, y, z] = QuadrotorPerformance(bestsol(i,:));
    trajX{i} = x; trajY{i} = y; trajZ{i} = z;

    fprintf('Run %d/%d finished | best fitness = %.6g | time = %s\n', ...
        i, NRuns, bestfitness(i), char(seconds(runTime(i))));

    avgRun = mean(runTime(1:i));
    etaSec = avgRun * (NRuns - i);
    fprintf('Estimated remaining time: %s\n', char(seconds(etaSec)));
end

%% === Best solution across runs ===
[globalBestFitness, bestRunIndex] = min(bestfitness);
best_gains = bestsol(bestRunIndex,:);

save('best_gains_TLBO.mat', 'best_gains', 'bestRunIndex', 'globalBestFitness');

%% === Summary Statistics ===
Stat = [min(bestfitness), max(bestfitness), mean(bestfitness), median(bestfitness), std(bestfitness)];
disp('Fitness Statistics: [min max mean median std]');
disp(Stat);

fprintf('Runtime per run (mean ± std): %.2f ± %.2f seconds\n', mean(runTime), std(runTime));

%% ==========================
%  End-of-simulation timing
%% ==========================
elapsedSec = toc(simTimer);
endTime = datetime('now');

fprintf('\nTLBO simulation completed at %s\n', datestr(endTime,'yyyy-mm-dd HH:MM:SS'));
fprintf('Total runtime: %s\n', char(seconds(elapsedSec)));

%% === Save everything explicitly ===
save('Tlbo_results.mat', ...
     'lb','ub','Np','T','NRuns', ...
     'bestsol','bestfitness','BestFitIter','trajX','trajY','trajZ', ...
     'runTime','best_gains','bestRunIndex','globalBestFitness','Stat', ...
     'startTime','endTime','elapsedSec');

%% ==========================
%  TLBO Results – Plots
%% ==========================
iters   = 0:T;
meanFit = mean(BestFitIter, 1);
stdFit  = std(BestFitIter, 0, 1);

%% Plot 1: Mean Convergence
figure('Color','w');
plot(iters, meanFit, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Average Best Fitness');
title('Mean Convergence (TLBO)');
grid on;

%% Plot 2: Mean ± Std Shaded
figure('Color','w');
fill([iters, fliplr(iters)], ...
     [meanFit + stdFit, fliplr(meanFit - stdFit)], ...
     [0.85 0.9 1], 'EdgeColor', 'none');
hold on;
plot(iters, meanFit, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Fitness');
title('Mean \pm Std of Convergence (TLBO)');
grid on;

%% Plot 3: Boxplot of final fitness
figure('Color','w');
boxplot(bestfitness, 'Notch','on');
ylabel('Final Best Fitness');
title(sprintf('Distribution of Final Fitness over %d Runs (TLBO)', NRuns));
grid on;

%% Plot 4: 3D Trajectory (best run)
best_x = trajX{bestRunIndex};
best_y = trajY{bestRunIndex};
best_z = trajZ{bestRunIndex};

figure('Color','w');
plot3(best_x, best_y, best_z, 'LineWidth', 2);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory of Best Run (TLBO)');
axis tight; view(35,22);

%% ==========================
%  Best TLBO evaluation (ONE run)
%  Control Effort + Attitude + Metrics CSV
%% ==========================
p = quad_params();

% Must match your updated signature:
% [Q, x, y, z, U, t, Tau, Att] = QuadrotorPerformance(gains)
[Qbest, xB, yB, zB, UB, tB, TauB, AttB] = QuadrotorPerformance(best_gains);

Tcmd     = TauB(:,1);
tauPhi   = TauB(:,2);
tauTheta = TauB(:,3);
tauPsi   = TauB(:,4);

phiB   = AttB(:,1);
thetaB = AttB(:,2);
psiB   = AttB(:,3);

% Save metrics (same function used by GA/PSO)
save_metrics_csv('TLBO', tB, zB, phiB, thetaB, psiB, TauB, UB, Qbest, p);

TLBO_summary = table( ...
    min(bestfitness), max(bestfitness), mean(bestfitness), median(bestfitness), std(bestfitness), ...
    mean(runTime), std(runTime), ...
    'VariableNames', {'Q_min','Q_max','Q_mean','Q_median','Q_std','runtime_mean_s','runtime_std_s'} );
writetable(TLBO_summary,'TLBO_summary_stats.csv');

%% --- Control effort plots (single plots) ---
figure('Color','w');
plot(tB, Tcmd, 'LineWidth',2); grid on; hold on;
yline(p.T_max,'--'); yline(0,'--');
xlabel('Time (s)'); ylabel('T_{cmd} (N)');
title('Best TLBO Solution: Thrust Command');

figure('Color','w');
plot(tB, tauPhi, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\phi} (N·m)');
title('Best TLBO Solution: Roll Torque');

figure('Color','w');
plot(tB, tauTheta, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\theta} (N·m)');
title('Best TLBO Solution: Pitch Torque');

figure('Color','w');
plot(tB, tauPsi, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\psi} (N·m)');
title('Best TLBO Solution: Yaw Torque');

%% --- Attitude tracking plots ---
figure('Color','w');
plot(tB, phiB, 'LineWidth',2); hold on; yline(p.roll_d,'--');
grid on; xlabel('Time (s)'); ylabel('\phi (rad)');
title('Best TLBO Solution: Roll Tracking');

figure('Color','w');
plot(tB, thetaB, 'LineWidth',2); hold on; yline(p.pitch_d,'--');
grid on; xlabel('Time (s)'); ylabel('\theta (rad)');
title('Best TLBO Solution: Pitch Tracking');

figure('Color','w');
plot(tB, psiB, 'LineWidth',2); hold on; yline(p.yaw_d,'--');
grid on; xlabel('Time (s)'); ylabel('\psi (rad)');
title('Best TLBO Solution: Yaw Tracking');

%% --- Altitude tracking plot ---
figure('Color','w');
plot(tB, zB, 'LineWidth',2); hold on; yline(p.z_d,'--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('z (m)');
title('Best TLBO Solution: Altitude Tracking');
