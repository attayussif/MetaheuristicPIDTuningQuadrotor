function [bestsol, bestfitness, BestFitIter, P, f, x, y, z] = DEbasic(prob, lb, ub, Np, T, F, pc)

    D = length(lb);                       % decision variables
    f = 1e6*ones(Np,1);                   % fitness (init penalized)
    fu = 1e6*ones(Np,1);                  % trial fitness
    U  = zeros(Np,D);                     % trial vectors
    P  = repmat(lb,Np,1) + repmat((ub-lb),Np,1).*rand(Np,D);  % init population

    % Keep trajectory logging empty here (match GA: do ONE sim/run in main script)
    x = []; y = []; z = [];

    BestFitIter = NaN(T+1,1);

    % ---------- Initial fitness ----------
    for i = 1:Np
        try
            f(i) = prob(P(i,:));
        catch
            f(i) = 1e6;
        end
    end

    [bestfitness, ind] = min(f);
    bestsol = P(ind,:);
    BestFitIter(1) = bestfitness;

    % ---------- Main DE loop ----------
    for t = 1:T
        fprintf('Iteration %3d / %3d | Current Best Fitness: %.6g\n', t, T, bestfitness);

        % --- Mutation + crossover (build trials U) ---
        for i = 1:Np
            Candidates = [1:i-1 i+1:Np];
            idx = Candidates(randperm(Np-1,3));
            X1 = P(idx(1),:);
            X2 = P(idx(2),:);
            X3 = P(idx(3),:);

            V = X1 + F*(X2 - X3);

            del = randi(D);  % force at least one mutant component
            for j = 1:D
                if (rand <= pc) || (j == del)
                    U(i,j) = V(j);
                else
                    U(i,j) = P(i,j);
                end
            end
        end

        % --- Bounding + selection ---
        for j = 1:Np
            U(j,:) = min(ub, max(lb, U(j,:)));  % clamp
            try
                fu(j) = prob(U(j,:));
            catch
                fu(j) = 1e6;
            end

            % greedy selection
            if fu(j) < f(j)
                P(j,:) = U(j,:);
                f(j)   = fu(j);
            end
        end

        % --- Update best (global) ---
        [genBest, ind] = min(f);
        if genBest < bestfitness
            bestfitness = genBest;
            bestsol = P(ind,:);
        end

        BestFitIter(t+1) = bestfitness;
    end

    % Keep format consistent with GA/PSO scripts:
    % If your DE main script expects row, leave it row.
    % If you *must* match your older "column bestsol" convention, uncomment:
    % bestsol = bestsol';

end
