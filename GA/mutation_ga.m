function child = mutation_ga(child, Pm, ub, lb, dim)

% random mutation
if rand < Pm
    for q = 1:dim
        if rand < 0.50
            child(q) = lb(q) + rand*(ub(q)-lb(q));
        end
    end
end

% Boundary Check
child = min(ub, max(lb, child));

end
