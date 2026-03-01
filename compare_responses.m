clc; clear; close all;

p = quad_params();

%% === Load best gains/controllers ===
load('PIDTUNE_Baseline_Gains.mat','gains');         best_PID = gains;
load('GA_best_gains.mat','best_gains');             best_GA  = best_gains;
load('PSO_best_gains.mat','best_gains');            best_PSO = best_gains;
load('DE_best_gains.mat','best_gains');             best_DE  = best_gains;
load('best_gains_TLBO.mat','best_gains');           best_TLBO= best_gains;
load('LQI_best_gains.mat','LQI');                   best_LQI = LQI;

T = array2table([best_PID; best_GA; best_PSO; best_DE; best_TLBO], ...
    'RowNames',{'PID','GA','PSO','DE','TLBO'});
writetable(T,'All_GAin_Metah_table.csv','WriteRowNames',true);
%% === Simulate each controller ===
[~,~,~,z_pid,~,t_pid,~,Att_pid] = QuadrotorPerformance(best_PID);
[~,~,~,z_ga ,~,t_ga ,~,Att_ga ] = QuadrotorPerformance(best_GA);
[~,~,~,z_pso,~,t_pso,~,Att_pso] = QuadrotorPerformance(best_PSO);
[~,~,~,z_de ,~,t_de ,~,Att_de ] = QuadrotorPerformance(best_DE);
[~,~,~,z_tl ,~,t_tl ,~,Att_tl ] = QuadrotorPerformance(best_TLBO);

[~,~,~,z_lqi,~,t_lqi,~,Att_lqi] = LQI_QuadrotorPerformance(best_LQI);

%% Extract attitudes
phi_pid = Att_pid(:,1); theta_pid = Att_pid(:,2); psi_pid = Att_pid(:,3);
phi_ga  = Att_ga(:,1);  theta_ga  = Att_ga(:,2);  psi_ga  = Att_ga(:,3);
phi_pso = Att_pso(:,1); theta_pso = Att_pso(:,2); psi_pso = Att_pso(:,3);
phi_de  = Att_de(:,1);  theta_de  = Att_de(:,2);  psi_de  = Att_de(:,3);
phi_tl  = Att_tl(:,1);  theta_tl  = Att_tl(:,2);  psi_tl  = Att_tl(:,3);
phi_lqi = Att_lqi(:,1); theta_lqi = Att_lqi(:,2); psi_lqi = Att_lqi(:,3);

%% ==========================
%  Altitude Comparison
%% ==========================
figure('Color','w');
plot(t_ga, z_ga,'LineWidth',2); hold on;
plot(t_pso,z_pso,'LineWidth',2);
plot(t_de, z_de,'LineWidth',2);
plot(t_tl, z_tl,'LineWidth',2);
plot(t_pid,z_pid,'LineWidth',2);
plot(t_lqi,z_lqi,'LineWidth',2);
yline(p.z_d,'k--','LineWidth',1.5);

grid on; axis tight;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Tracking Comparison');
legend('GA','PSO','DE','TLBO','PID','LQI','Reference','Location','best');

exportgraphics(gcf,'altitude_comparison.png','Resolution',400);

%% ==========================
%  Roll Comparison
%% ==========================
figure('Color','w');
plot(t_ga, phi_ga,'LineWidth',2); hold on;
plot(t_pso,phi_pso,'LineWidth',2);
plot(t_de, phi_de,'LineWidth',2);
plot(t_tl, phi_tl,'LineWidth',2);
plot(t_pid,phi_pid,'LineWidth',2);
plot(t_lqi,phi_lqi,'LineWidth',2);
yline(p.roll_d,'k--','LineWidth',1.5);

grid on; axis tight;
xlabel('Time (s)');
ylabel('\phi (rad)');
title('Roll Angle Comparison');
legend('GA','PSO','DE','TLBO','PID','LQI','Reference','Location','best');

exportgraphics(gcf,'roll_comparison.png','Resolution',400);

%% ==========================
%  Pitch Comparison
%% ==========================
figure('Color','w');
plot(t_ga, theta_ga,'LineWidth',2); hold on;
plot(t_pso,theta_pso,'LineWidth',2);
plot(t_de, theta_de,'LineWidth',2);
plot(t_tl, theta_tl,'LineWidth',2);
plot(t_pid,theta_pid,'LineWidth',2);
plot(t_lqi,theta_lqi,'LineWidth',2);
yline(p.pitch_d,'k--','LineWidth',1.5);

grid on; axis tight;
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Pitch Angle Comparison');
legend('GA','PSO','DE','TLBO','PID','LQI','Reference','Location','best');

exportgraphics(gcf,'pitch_comparison.png','Resolution',400);

%% ==========================
%  Yaw Comparison
%% ==========================
figure('Color','w');
plot(t_ga, psi_ga,'LineWidth',2); hold on;
plot(t_pso,psi_pso,'LineWidth',2);
plot(t_de, psi_de,'LineWidth',2);
plot(t_tl, psi_tl,'LineWidth',2);
plot(t_pid,psi_pid,'LineWidth',2);
plot(t_lqi,psi_lqi,'LineWidth',2);
yline(p.yaw_d,'k--','LineWidth',1.5);

grid on; axis tight;
xlabel('Time (s)');
ylabel('\psi (rad)');
title('Yaw Angle Comparison');
legend('GA','PSO','DE','TLBO','PID','LQI','Reference','Location','best');

exportgraphics(gcf,'yaw_comparison.png','Resolution',400);