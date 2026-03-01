%% stats_compare_Q.m
clc; clear; close all;

% --- Load bestfitness from each algorithm result file ---
GA   = load('Ga_results.mat');    Q_GA   = GA.bestfitness(:);
PSO  = load('pso_results.mat');   Q_PSO  = PSO.bestfitness(:);
DE   = load('De_results.mat');    Q_DE   = DE.bestfitness(:);
TLBO = load('Tlbo_results.mat');  Q_TLBO = TLBO.bestfitness(:);

% --- Stack data for ANOVA ---
Q = [Q_GA; Q_PSO; Q_DE; Q_TLBO];
group = [repmat("GA",numel(Q_GA),1);
         repmat("PSO",numel(Q_PSO),1);
         repmat("DE",numel(Q_DE),1);
         repmat("TLBO",numel(Q_TLBO),1)];

% --- One-way ANOVA (suppress figure) ---
[p_anova, tbl, stats] = anova1(Q, group, 'off');

% Extract ANOVA components
SS_between = tbl{2,2};
SS_within  = tbl{3,2};
SS_total   = tbl{4,2};
df_between = tbl{2,3};
df_within  = tbl{3,3};
F_stat     = tbl{2,5};

% --- Effect size: eta^2 and omega^2 ---
eta2   = SS_between / SS_total;
omega2 = (SS_between - df_between*(SS_within/df_within)) / (SS_total + (SS_within/df_within));

% --- Tukey HSD post-hoc (multcompare uses Tukey-Kramer) ---
C = multcompare(stats, 'CType','tukey-kramer', 'Display','off');
% C columns: [i j lower meanDiff upper pValue]
groups = stats.gnames;

% --- Pairwise Hedges g (optional but strong for reviewer) ---
pairs = nchoosek(1:numel(groups),2);
hedges = table('Size',[size(pairs,1) 4], ...
               'VariableTypes',{'string','string','double','double'}, ...
               'VariableNames',{'A','B','g','p_ttest'});
for k=1:size(pairs,1)
    i = pairs(k,1); j = pairs(k,2);
    Qi = Q(group==groups{i});
    Qj = Q(group==groups{j});
    
    % Hedges g
    ni = numel(Qi); nj = numel(Qj);
    si = var(Qi,1); sj = var(Qj,1);
    sp = sqrt(((ni-1)*var(Qi,0) + (nj-1)*var(Qj,0)) / (ni+nj-2));
    d  = (mean(Qi)-mean(Qj))/sp;
    J  = 1 - (3/(4*(ni+nj)-9)); % correction
    g  = J*d;
    
    % Welch t-test p-value
    [~,p_tt] = ttest2(Qi,Qj,'Vartype','unequal');
    
    hedges.A(k) = groups{i};
    hedges.B(k) = groups{j};
    hedges.g(k) = g;
    hedges.p_ttest(k) = p_tt;
end

% --- Descriptive table with CI ---
algNames = ["GA","PSO","DE","TLBO"];
Qcell = {Q_GA,Q_PSO,Q_DE,Q_TLBO};

desc = table('Size',[4 7], ...
    'VariableTypes',{'string','double','double','double','double','double','string'}, ...
    'VariableNames',{'Alg','n','Qmin','Qmed','Qmean','Qstd','CI95'});

for i=1:4
    Qi = Qcell{i};
    n  = numel(Qi);
    mu = mean(Qi);
    sd = std(Qi);
    tcrit = tinv(0.975,n-1);
    ciL = mu - tcrit*sd/sqrt(n);
    ciU = mu + tcrit*sd/sqrt(n);
    
    desc.Alg(i)   = algNames(i);
    desc.n(i)     = n;
    desc.Qmin(i)  = min(Qi);
    desc.Qmed(i)  = median(Qi);
    desc.Qmean(i) = mu;
    desc.Qstd(i)  = sd;
    desc.CI95(i)  = sprintf("[%.6f, %.6f]", ciL, ciU);
end

% --- Print key outputs ---
fprintf("ANOVA: F(%d,%d)=%.6f, p=%.3e, eta^2=%.4f, omega^2=%.4f\n", ...
    df_between, df_within, F_stat, p_anova, eta2, omega2);

disp("Descriptive stats:"); disp(desc);
disp("Tukey HSD table [i j lower meanDiff upper p]:"); disp(array2table(C, ...
    'VariableNames',{'i','j','lower','meanDiff','upper','p'}));
disp("Pairwise Hedges g and Welch t-test p:"); disp(hedges);

% --- Save tables for LaTeX/Excel use ---
writetable(desc,   'Q_descriptive_stats.csv');
writetable(array2table(C,'VariableNames',{'i','j','lower','meanDiff','upper','p'}), 'Q_tukey.csv');
writetable(hedges, 'Q_pairwise_hedgesg.csv');