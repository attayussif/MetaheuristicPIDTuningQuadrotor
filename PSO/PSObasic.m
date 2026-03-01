function [bestsol, bestfitness, BestFitIter, P, f, x, y, z] = PSObasic(prob, lb, ub, Np, T, w, c1, c2)

D = length(lb);

f = zeros(Np,1);
BestFitIter = zeros(T+1,1);

% --- Initialize population ---
P = repmat(lb, Np, 1) + repmat((ub-lb), Np, 1) .* rand(Np, D);

% --- Seed the first particle with PID-tuner gains (ODE scale) ---
seed = [ ...
    0.1069  0.0062  0.4618  ...  % z
    0.0017  0.0001  0.0074  ...  % roll
    0.0017  0.0001  0.0074  ...  % pitch
    0.0030  0.0002  0.0128  ...  % yaw
];
seed = min(ub, max(lb, seed));
P(1,:) = seed;

% --- Velocity ---
v = zeros(Np, D);

% --- Initial Evaluation (fitness only) ---
for i = 1:Np
    try
        f(i) = prob(P(i,:));   % uses only Q
    catch
        f(i) = 1e6;
    end
end

% --- Personal bests ---
pbest = P;
f_pbest = f;

% --- Global best ---
[bestfitness, ind] = min(f);
gbest = P(ind,:);

% --- Get trajectory for the initial gbest (one call) ---
try
    [~, x, y, z] = prob(gbest);
catch
    x = []; y = []; z = [];
end

BestFitIter(1) = bestfitness;

% --- Main Loop ---
for t = 1:T
    fprintf('Iteration %3d / %3d | Current Best Fitness: %.6g\n', t, T, bestfitness);

    for i = 1:Np
        % Velocity update
        v(i,:) = w*v(i,:) + ...
                 c1*rand(1,D).*(pbest(i,:) - P(i,:)) + ...
                 c2*rand(1,D).*(gbest      - P(i,:));

        % Position update + bounds
        P(i,:) = P(i,:) + v(i,:);
        P(i,:) = min(ub, max(lb, P(i,:)));

        % Fitness evaluation (fitness only)
        try
            fi = prob(P(i,:));
        catch
            fi = 1e6;
        end
        f(i) = fi;

        % Update pbest
        if fi < f_pbest(i)
            pbest(i,:) = P(i,:);
            f_pbest(i) = fi;

            % Update gbest
            if fi < bestfitness
                bestfitness = fi;
                gbest = P(i,:);

                % Update trajectory only when a new global best is found
                try
                    [~, x, y, z] = QuadrotorPerformance(gbest);
                catch
                    x = []; y = []; z = [];
                end
            end
        end
    end

    BestFitIter(t+1) = bestfitness;
end

bestsol = gbest;   % keep as row vector to match your PSO main script
end
