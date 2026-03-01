function [ parent1, parent2] = selection_ga(population,fitness)

parent = zeros(2,size(population,2));

for ii=1:2
    k=randi(size(population,1));
    kk=randi(size(population,1));

    % Tournament selection
    if rand<0.90
        if fitness(k)<=fitness(kk)
            parent(ii,:)=population(k,:);
        else
            parent(ii,:)=population(kk,:);
        end
        parent1=parent(1,:);
        parent2=parent(2,:);
    else
        if fitness(k)>=fitness(kk)
            parent(ii,:)=population(k,:);
        else
            parent(ii,:)=population(kk,:);
        end
        parent1=parent(1,:);
        parent2=parent(2,:);
    end
end