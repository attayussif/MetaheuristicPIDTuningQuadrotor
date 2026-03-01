function [child1, child2] = crossover_ga(parent1, parent2, Pc)

% BLX-alpha cross over
alpha = 0.50;

if rand < Pc
    c_min = min([parent1; parent2], [], 1);
    c_max = max([parent1; parent2], [], 1);
    I = c_max - c_min;

    cross_min = c_min - I*alpha;
    cross_max = c_max + I*alpha;

    child1 = cross_min + rand(1, numel(parent1)).*(cross_max - cross_min);
    child2 = cross_min + rand(1, numel(parent2)).*(cross_max - cross_min);
else
    child1 = parent1;
    child2 = parent2;
end

end
