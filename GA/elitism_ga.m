function pop_elt = elitism_ga(population, fitness, no_elt)

pop_elt = zeros(no_elt, size(population,2));

for p = 1:no_elt
    [~, best_obj_ind] = min(fitness);
    pop_elt(p,:) = population(best_obj_ind,:);
    population(best_obj_ind,:) = [];
    fitness(best_obj_ind,:) = [];
end

end
