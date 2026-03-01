function [bestsol, bestfitness, BestFitIter, P, f] = TLBObasic(prob, lb, ub, Np, T)
%TLBOBASIC  Teaching–Learning-Based Optimization (ODE-only fitness)
%   prob : function handle, returns scalar cost Q (minimize)
%   lb,ub: 1xD bounds
%   Np   : population size
%   T    : iterations
%
% Outputs:
%   bestsol     : 1xD best decision vector (ROW)
%   bestfitness : scalar
%   BestFitIter : 1x(T+1) best-so-far history (ROW)
%   P           : Np x D final population
%   f           : Np x 1 final fitness values

    D = length(lb);

    % --- Initialize population ---
    P = repmat(lb, Np, 1) + rand(Np, D) .* repmat((ub - lb), Np, 1);
    f = NaN(Np, 1);

    % --- Seed one known-good solution (PID tuner baseline) ---
    seed = [ ...
        0.1069  0.0062  0.4618  ...  % z
        0.0017  0.0001  0.0074  ...  % roll
        0.0017  0.0001  0.0074  ...  % pitch
        0.0030  0.0002  0.0128  ...  % yaw
    ];
    
    seed = min(ub, max(lb, seed));   % clamp to bounds
    P(1,:) = seed;                   % inject seed as first learner


    % Store best-so-far history (row to match your scripts)
    BestFitIter = NaN(1, T+1);

    % --- Initial evaluation ---
    for i = 1:Np
        try
            f(i) = prob(P(i, :));
        catch
            f(i) = 1e6;
        end
    end

    [bestfitness, best_idx] = min(f);
    bestsol = P(best_idx, :);         % 1xD row
    BestFitIter(1) = bestfitness;

    % === Main TLBO Loop ===
    for t = 1:T

        % --------------------------
        % Teacher Phase
        % --------------------------
        [~, teacher_idx] = min(f);
        Teacher = P(teacher_idx, :);

        Tf = randi([1, 2]);           % Teaching factor: 1 or 2
        mean_P = mean(P, 1);          % 1xD

        for i = 1:Np
            new_solution = P(i,:) + rand(1, D) .* (Teacher - Tf * mean_P);
            new_solution = min(ub, max(lb, new_solution));  % bounds

            try
                f_new = prob(new_solution);
            catch
                f_new = 1e6;
            end

            if f_new < f(i)
                P(i,:) = new_solution;
                f(i) = f_new;

                % Update global best
                if f_new < bestfitness
                    bestfitness = f_new;
                    bestsol = new_solution;
                end
            end
        end

        % --------------------------
        % Learner Phase
        % --------------------------
        for i = 1:Np
            j = randi(Np);
            while j == i
                j = randi(Np);
            end

            if f(i) < f(j)
                new_solution = P(i,:) + rand(1, D) .* (P(i,:) - P(j,:));
            else
                new_solution = P(i,:) + rand(1, D) .* (P(j,:) - P(i,:));
            end

            new_solution = min(ub, max(lb, new_solution));  % bounds

            try
                f_new = prob(new_solution);
            catch
                f_new = 1e6;
            end

            if f_new < f(i)
                P(i,:) = new_solution;
                f(i) = f_new;

                % Update global best
                if f_new < bestfitness
                    bestfitness = f_new;
                    bestsol = new_solution;
                end
            end
        end

        BestFitIter(t + 1) = bestfitness;
        fprintf('Iteration %3d / %3d | Current Best Fitness: %.6g\n', t, T, bestfitness);
    end
end
