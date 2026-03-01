function [bestsol, bestfitness, BestFitIter, P, f, x, y, z] = GAbasic(prob, lb, ub, Np, T, Pc, Pm, elitism_num)

D = length(lb);                   % Number of decision variables
P = repmat(lb, Np, 1) + repmat((ub - lb), Np, 1) .* rand(Np, D); % Initial population
f = NaN(Np,1);                    % Fitness of each individual

x = []; y = []; z = [];          % For best trajectory logging (kept empty here)
BestFitIter = NaN(T+1,1);        % Fitness log

% === Seed one known-good individual (DO THIS BEFORE fitness evaluation) ===
manual_gains = [ ...
    0.1069	0.0062	0.4618  ...  % z
    0.0017	0.0001	0.0074  ...  % roll
    0.0017	0.0001	0.0074  ...  % pitch
    0.0030	0.0002	0.0128 ...  % yaw
];
manual_gains = min(ub, max(lb, manual_gains));
P(1,:) = manual_gains;

% === Initial Fitness Evaluation ===
for i = 1:Np
    try
        f(i) = prob(P(i,:));
    catch
        f(i) = 1e6;  % Penalize failed runs
    end
end

% === Store Initial Best ===
[bestfitness, ind] = min(f);
bestsol = P(ind,:);
BestFitIter(1) = bestfitness;

% === Main GA Loop ===
for t = 1:T

    % --- Safety: ensure elitism does not kill reproduction ---
    elitism_num_eff = min(elitism_num, Np-2);   % keep at least 2 children

    % --- Elitism ---
    pop_elt = elitism_ga(P, f, elitism_num_eff);

    % --- Mating Pool ---
    nChildren = Np - elitism_num_eff;
    children  = zeros(nChildren, D);

    k = 1;
    while k <= nChildren
        % Parent selection (Tournament selection, minimization)
        tourSize = 3;

        idx1 = randperm(Np, tourSize);
        [~, b1] = min(f(idx1));
        parent1 = P(idx1(b1), :);

        idx2 = randperm(Np, tourSize);
        [~, b2] = min(f(idx2));
        parent2 = P(idx2(b2), :);

        % Crossover
        [child1, child2] = crossover_ga(parent1, parent2, Pc);

        % Mutation
        child1 = mutation_ga(child1, Pm, ub, lb, D);
        child2 = mutation_ga(child2, Pm, ub, lb, D);

        % Hard bound (safety, reduces sim failures)
        child1 = min(ub, max(lb, child1));
        child2 = min(ub, max(lb, child2));

        % Add to next generation
        children(k,:) = child1;
        if k+1 <= nChildren
            children(k+1,:) = child2;
        end
        k = k + 2;
    end

    % --- Next Generation ---
    P = [pop_elt; children];      % stays Np individuals

    % --- Evaluate Fitness ---
    for i = 1:Np
        try
            f(i) = prob(P(i,:));
        catch
            f(i) = 1e6;
        end
    end

    % Update global best from this generation (robust)
    [genBest, ind] = min(f);
    if genBest < bestfitness
        bestfitness = genBest;
        bestsol = P(ind,:);
    end

    % --- Log best of iteration ---
    BestFitIter(t+1) = bestfitness;

    % === Progress print (every generation) ===
    fprintf('      Gen %3d/%3d | best = %.6g\n', t, T, bestfitness);
end

bestsol = bestsol';  % Match format with other optimizers

end
