clc; clear; close all;

%% ==========================
%  Problem Setup (bounds)
%% ==========================
lb = [ ...
  0.02   1e-5   0.02,  ...  % z
  1e-4   1e-6   1e-4,  ...  % phi
  1e-4   1e-6   1e-4,  ...  % theta
  1e-4   1e-6   1e-4];      % psi

ub = [ ...
  2.0    0.2    2.0,   ...  % z
  0.3    0.05   0.3,   ...  % phi
  0.3    0.05   0.3,   ...  % theta
  0.4    0.05   0.4];       % psi

prob = @(g) QuadrotorPerformance(g);   % MATLAB ODE objective

%% ==========================
%  DE Parameters (edit here)
%% ==========================
Np    = 15;     % population size
T     = 60;     % generations/iterations
F     = 0.7;    % mutation factor (typical 0.4..0.9)
Cr    = 0.9;    % crossover rate (typical 0.7..0.95)
NRuns = 100;     % independent runs

%% ==========================
%  Timing / runtime tracking
%% ==========================
startTime = datetime('now');
simTimer  = tic;
fprintf('DE simulation started at %s\n', datestr(startTime,'yyyy-mm-dd HH:MM:SS'));

%% ==========================
%  Storage
%% ==========================
D = numel(lb);

bestsol     = zeros(NRuns, D);
bestfitness = zeros(NRuns, 1);
BestFitIter = zeros(NRuns, T+1);

trajX = cell(NRuns,1);
trajY = cell(NRuns,1);
trajZ = cell(NRuns,1);

runTime = zeros(NRuns,1);

%% ==========================
%  DE Optimization Loop
%% ==========================
for i = 1:NRuns
    fprintf('\nDE Run %d\n', i);
    rng(i,'twister');

    tRun = tic;

    % --- DE core ---
    % EXPECTED: [best_i, fbest_i, fitlog_i, P, f, x, y, z] = DEbasic(...)
    [best_i, fbest_i, fitlog_i, ~, ~, x, y, z] = DEbasic(prob, lb, ub, Np, T, F, Cr);

    runTime(i) = toc(tRun);

    bestsol(i,:)     = best_i(:).';     % ensure row
    bestfitness(i)   = fbest_i;
    BestFitIter(i,:) = fitlog_i(:).';   % ensure row length T+1

    trajX{i} = x; trajY{i} = y; trajZ{i} = z;

    fprintf('Run %d/%d finished | best fitness = %.6g | time = %s\n', ...
        i, NRuns, bestfitness(i), char(seconds(runTime(i))));

    avgRun = mean(runTime(1:i));
    etaSec = avgRun * (NRuns - i);
    fprintf('Estimated remaining time: %s\n', char(seconds(etaSec)));
end

%% ==========================
%  Best solution across runs
%% ==========================
[globalBestFitness, bestRunIndex] = min(bestfitness);
best_gains = bestsol(bestRunIndex,:);

save('best_gains_DE.mat', 'best_gains', 'bestRunIndex', 'globalBestFitness');

%% ==========================
%  Summary statistics
%% ==========================
Stat = [min(bestfitness), max(bestfitness), mean(bestfitness), median(bestfitness), std(bestfitness)];
disp('Fitness Statistics: [min max mean median std]');
disp(Stat);

fprintf('Runtime per run (mean ± std): %.2f ± %.2f seconds\n', mean(runTime), std(runTime));

%% ==========================
%  End-of-simulation timing
%% ==========================
elapsedSec = toc(simTimer);
endTime = datetime('now');

fprintf('\nDE simulation completed at %s\n', datestr(endTime,'yyyy-mm-dd HH:MM:SS'));
fprintf('Total runtime: %s\n', char(seconds(elapsedSec)));

%% ==========================
%  Save results mat (robust)
%% ==========================
save('De_results.mat', ...
     'lb','ub','Np','T','F','Cr','NRuns', ...
     'bestsol','bestfitness','BestFitIter','trajX','trajY','trajZ', ...
     'runTime','best_gains','bestRunIndex','globalBestFitness','Stat', ...
     'startTime','endTime','elapsedSec');

%% ==========================
%  Plots 
%% ==========================
iters   = 0:(size(BestFitIter,2)-1);
meanFit = mean(BestFitIter, 1);
stdFit  = std(BestFitIter, 0, 1);

% Plot 1: Mean convergence
figure('Color','w');
plot(iters, meanFit, 'LineWidth', 2);
xlabel('Generation'); ylabel('Average Best Fitness');
title('Mean Convergence (DE)');
grid on;

% Plot 2: Mean ± Std shaded
figure('Color','w');
fill([iters fliplr(iters)], ...
     [meanFit + stdFit, fliplr(meanFit - stdFit)], ...
     [0.85 0.9 1], 'EdgeColor','none');
hold on;
plot(iters, meanFit, 'LineWidth', 2);
xlabel('Generation'); ylabel('Fitness');
title('Mean \pm Std of Convergence (DE)');
grid on;

% Plot 3: Boxplot
figure('Color','w');
boxplot(bestfitness,'Notch','on');
ylabel('Final Best Fitness');
title(sprintf('Distribution of Final Fitness over %d Runs (DE)', NRuns));
grid on;

% Plot 4: Best trajectory (3D)
best_x = trajX{bestRunIndex};
best_y = trajY{bestRunIndex};
best_z = trajZ{bestRunIndex};

figure('Color','w');
plot3(best_x, best_y, best_z, 'LineWidth', 2);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory of Best Run (DE)');
axis tight; view(35,22);

%% ==========================
%  Best DE evaluation (ONE sim)
%  Control effort + attitude + altitude + metrics CSV
%% ==========================
p = quad_params();

[Qbest, xB, yB, zB, UB, tB, TauB, AttB] = QuadrotorPerformance(best_gains);

Tcmd     = TauB(:,1);
tauPhi   = TauB(:,2);
tauTheta = TauB(:,3);
tauPsi   = TauB(:,4);

phiB   = AttB(:,1);
thetaB = AttB(:,2);
psiB   = AttB(:,3);

% Metrics CSV
save_metrics_csv('DE', tB, zB, phiB, thetaB, psiB, TauB, UB, Qbest, p);

DE_summary = table( ...
    min(bestfitness), max(bestfitness), mean(bestfitness), median(bestfitness), std(bestfitness), ...
    mean(runTime), std(runTime), ...
    'VariableNames', {'Q_min','Q_max','Q_mean','Q_median','Q_std','runtime_mean_s','runtime_std_s'} );
writetable(DE_summary,'DE_summary_stats.csv');

%% --- Control effort plots (single plots, matching GA/PSO) ---
figure('Color','w');
plot(tB, Tcmd, 'LineWidth',2); grid on; hold on;
yline(p.T_max,'--'); yline(0,'--');
xlabel('Time (s)'); ylabel('T_{cmd} (N)');
title('Best DE Solution: Thrust Command');

figure('Color','w');
plot(tB, tauPhi, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\phi} (N·m)');
title('Best DE Solution: Roll Torque');

figure('Color','w');
plot(tB, tauTheta, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\theta} (N·m)');
title('Best DE Solution: Pitch Torque');

figure('Color','w');
plot(tB, tauPsi, 'LineWidth',2); grid on; hold on;
yline(p.tau_max,'--'); yline(-p.tau_max,'--');
xlabel('Time (s)'); ylabel('\tau_{\psi} (N·m)');
title('Best DE Solution: Yaw Torque');

%% --- Attitude tracking (single plots) ---
figure('Color','w');
plot(tB, phiB, 'LineWidth',2); hold on; yline(p.roll_d,'--');
grid on; xlabel('Time (s)'); ylabel('\phi (rad)');
title('Best DE Solution: Roll Tracking');

figure('Color','w');
plot(tB, thetaB, 'LineWidth',2); hold on; yline(p.pitch_d,'--');
grid on; xlabel('Time (s)'); ylabel('\theta (rad)');
title('Best DE Solution: Pitch Tracking');

figure('Color','w');
plot(tB, psiB, 'LineWidth',2); hold on; yline(p.yaw_d,'--');
grid on; xlabel('Time (s)'); ylabel('\psi (rad)');
title('Best DE Solution: Yaw Tracking');

%% --- Altitude tracking ---
figure('Color','w');
plot(tB, zB, 'LineWidth',2); hold on; yline(p.z_d,'--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('z (m)');
title('Best DE Solution: Altitude Tracking');
