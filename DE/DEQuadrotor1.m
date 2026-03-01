clear;
close all;

%% === Problem Setup ===

% Lower and upper bounds for 12 PID gains [Kp_z Ki_z Kd_z ... Kd_psi]
lb = [30  0.001  0.5, ...
      30  0.001  5, ...
      30  0.001  10, ...
      30  0.001  0.5];

ub = [100  0.1    30, ...
      100  0.1    30, ...
      100  0.1    30, ...
      100  0.1    30];

prob = @QuadrotorPerformance;  % Objective function (unchanged)

%% === DE Algorithm Parameters ===
Np = 15;           % Population size
T = 30;            % Max generations
F = 0.85;          % Scaling factor (mutation)
pc = 0.8;          % Crossover probability
NRuns = 20;         % Number of independent runs

%% === Storage Initialization ===
bestsol = zeros(NRuns, length(lb));
bestfitness = zeros(NRuns, 1);
BestFitIter = zeros(NRuns, T+1);

%% === DE Optimization Loop ===
for i = 1:NRuns
    disp(['DE Run ', num2str(i)]);
    rng(i, 'twister');  % Reproducible random seed

    [bestsol(i,:), bestfitness(i), BestFitIter(i,:), ~, ~, x, y, z] = ...
        DEbasic(prob, lb, ub, Np, T, F, pc);
end

% Save best gains
best_gains = bestsol(end,:);  % Or use min(bestfitness) index
save('best_gains.mat', 'best_gains');

%% === Summary Statistics ===
Stat(1) = min(bestfitness);
Stat(2) = max(bestfitness);
Stat(3) = mean(bestfitness);
Stat(4) = median(bestfitness);
Stat(5) = std(bestfitness);

disp('Fitness Statistics: [min max mean median std]');
disp(Stat);

%% === Plot 1: Mean Convergence ===
meanFit = mean(BestFitIter);
figure;
plot(0:T, meanFit, 'b', 'LineWidth', 2);
xlabel('Iteration'); ylabel('Average Best Fitness');
title('Mean Convergence (DE)');
grid on;

%% === Plot 2: Mean ± Std Shaded Area ===
stdFit = std(BestFitIter);
iters = 0:T;
figure;
fill([iters, fliplr(iters)], ...
     [meanFit + stdFit, fliplr(meanFit - stdFit)], ...
     [0.85 0.9 1], 'EdgeColor', 'none'); hold on;
plot(iters, meanFit, 'b', 'LineWidth', 2);
xlabel('Iteration'); ylabel('Fitness');
title('Mean ± Std of Convergence (DE)');
grid on;

%% === Plot 3: Boxplot of Final Fitness ===
figure;
boxplot(bestfitness, 'Notch', 'on');
ylabel('Final Best Fitness');
title('Distribution of Final Fitness over Runs (DE)');
grid on;

%% === Plot 4: 3D Trajectory of Best Run ===
[~, bestRunIndex] = min(bestfitness);
best_x = x;
best_y = y;
best_z = z;

figure;
plot3(best_x, best_y, best_z, 'b', 'LineWidth', 2);
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory of Best Run');

save("DE_Results_workspace.mat")
%%
%Mean convergence
iters   = 0:T;
N       = size(BestFitIter,1);
meanFit = mean(BestFitIter, 1);
stdFit  = std(BestFitIter, 0, 1);
semFit  = stdFit / sqrt(N);
ci95    = 1.96 * semFit;   % approx 95% CI (normal approx)

fig = figure('Color','w','Units','pixels','Position',[100 100 820 420]);
ax = axes(fig); hold(ax,'on');

upper = meanFit + ci95;
lower = meanFit - ci95;

p = fill([iters fliplr(iters)], [upper fliplr(lower)], ...
         [0.2 0.45 0.9], 'EdgeColor','none');
p.FaceAlpha = 0.18;

plot(iters, meanFit, 'Color',[0.05 0.25 0.85], 'LineWidth', 2.5);

grid(ax,'on'); ax.GridAlpha = 0.15; ax.LineWidth = 1.1;
ax.FontName = 'Times New Roman'; ax.FontSize = 13;
xlabel('Iteration','FontSize',14);
ylabel('Best-so-far fitness','FontSize',14);
title(sprintf('DE Convergence (N = %d runs): Mean \\pm 95%% CI', N), ...
      'FontSize',14, 'FontWeight','normal');

lg = legend({'95% CI band','Mean'}, 'Location','northeast');
lg.Box = 'off';
xlim([0 T]); axis(ax,'tight');

exportgraphics(fig, 'DE_mean_ci95.png', 'Resolution', 400);


% Boxplot of final fitness
fig = figure('Color','w','Units','pixels','Position',[100 100 520 420]);
ax = axes(fig);

boxplot(bestfitness(:), 'Notch','on', 'Symbol','k.');
grid(ax,'on'); ax.GridAlpha = 0.15; ax.LineWidth = 1.1;
ax.FontName = 'Times New Roman'; ax.FontSize = 13;

ylabel('Final best fitness','FontSize',14);
title(sprintf('Final Fitness Distribution (N = %d runs)', numel(bestfitness)), ...
      'FontSize',14, 'FontWeight','normal');

exportgraphics(fig, 'DE_final_boxplot.png', 'Resolution', 400);

% 3D best-run trajectory

[~, bestRunIndex] = min(bestfitness);

best_x = x(:, bestRunIndex);
best_y = y(:, bestRunIndex);
best_z = z(:, bestRunIndex);

fig = figure('Color','w','Units','pixels','Position',[100 100 650 520]);
ax = axes(fig); hold(ax,'on');

plot3(best_x, best_y, best_z, 'LineWidth', 2.2);
grid(ax,'on'); ax.GridAlpha = 0.15; ax.LineWidth = 1.1;
ax.FontName = 'Times New Roman'; ax.FontSize = 13;

xlabel('X (m)','FontSize',14);
ylabel('Y (m)','FontSize',14);
zlabel('Z (m)','FontSize',14);
title('3D Trajectory of Best Run (DE)','FontSize',14,'FontWeight','normal');

axis(ax,'tight');
view(35, 22);

exportgraphics(fig, 'DE_best_trajectory_3D.png', 'Resolution', 400);
