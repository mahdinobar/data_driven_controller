function plot_results_2
% hyper-params
N_iter=50;
repeat_experiment=250;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

% =========================================================================
dir='/home/mahdi/PhD application/ETH/Rupenyan/code/results/server_runs_ver_3';
tmp_N03=[];
tmp_N05=[];
tmp_N010=[];
tmp_N020=[];
for m=1:4
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N03_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N03_tmp=reshape(objectiveData_all_server_N03_tmp(1:end),[50,250]);
    tmp_N03=[tmp_N03, objectiveData_all_server_N03_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+4),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N05_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N05_tmp=reshape(objectiveData_all_server_N05_tmp(1:end),[50,250]);
    tmp_N05=[tmp_N05, objectiveData_all_server_N05_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+8),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N010_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N010_tmp=reshape(objectiveData_all_server_N010_tmp(1:end),[50,250]);
    tmp_N010=[tmp_N010, objectiveData_all_server_N010_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+12),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N020_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N020_tmp=reshape(objectiveData_all_server_N020_tmp(1:end),[50,250]);
    tmp_N020=[tmp_N020, objectiveData_all_server_N020_tmp];
end
objectiveData_all_server_N03=tmp_N03;
objectiveData_all_server_N05=tmp_N05;
objectiveData_all_server_N010=tmp_N010;
objectiveData_all_server_N020=tmp_N020;


for i=1:1000
    for j=1:50
        objectiveData_all_server_N03(j,i)=nanmin(objectiveData_all_server_N03(1:j,i));
        objectiveData_all_server_N05(j,i)=nanmin(objectiveData_all_server_N05(1:j,i));
        objectiveData_all_server_N010(j,i)=nanmin(objectiveData_all_server_N010(1:j,i));
        objectiveData_all_server_N020(j,i)=nanmin(objectiveData_all_server_N020(1:j,i));
%         objectiveData_all_server_surrogate(j,i)=nanmin(objectiveData_all_server_surrogate(1:j,i));
    end
end

mean_objectiveData_all_server_N03=mean(objectiveData_all_server_N03,2,'omitnan');
mean_objectiveData_all_server_N05=mean(objectiveData_all_server_N05,2,'omitnan');
mean_objectiveData_all_server_N010=mean(objectiveData_all_server_N010,2,'omitnan');
mean_objectiveData_all_server_N020=mean(objectiveData_all_server_N020,2,'omitnan');
% =========================================================================
dir='/home/mahdi/PhD application/ETH/Rupenyan/code/results/server_runs_ver_3_2';
tmp_N03=[];
tmp_N05=[];
tmp_N010=[];
tmp_N020=[];
for m=17:20
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N03_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N03_tmp=reshape(objectiveData_all_server_N03_tmp(1:end),[50,250]);
    tmp_N03=[tmp_N03, objectiveData_all_server_N03_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+4),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N05_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N05_tmp=reshape(objectiveData_all_server_N05_tmp(1:end),[50,250]);
    tmp_N05=[tmp_N05, objectiveData_all_server_N05_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+8),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N010_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N010_tmp=reshape(objectiveData_all_server_N010_tmp(1:end),[50,250]);
    tmp_N010=[tmp_N010, objectiveData_all_server_N010_tmp];
    
    load(append(dir,'/InitobjectiveData_all_',num2str(m+12),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_N020_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_N020_tmp=reshape(objectiveData_all_server_N020_tmp(1:end),[50,250]);
    tmp_N020=[tmp_N020, objectiveData_all_server_N020_tmp];
end
objectiveData_all_server_N03=tmp_N03;
objectiveData_all_server_N05=tmp_N05;
objectiveData_all_server_N010=tmp_N010;
objectiveData_all_server_N020=tmp_N020;


for i=1:1000
    for j=1:50
        objectiveData_all_server_N03(j,i)=nanmin(objectiveData_all_server_N03(1:j,i));
        objectiveData_all_server_N05(j,i)=nanmin(objectiveData_all_server_N05(1:j,i));
        objectiveData_all_server_N010(j,i)=nanmin(objectiveData_all_server_N010(1:j,i));
        objectiveData_all_server_N020(j,i)=nanmin(objectiveData_all_server_N020(1:j,i));
%         objectiveData_all_server_surrogate(j,i)=nanmin(objectiveData_all_server_surrogate(1:j,i));
    end
end

mean_objectiveData_all_server_N03_GBO=mean(objectiveData_all_server_N03,2,'omitnan');
mean_objectiveData_all_server_N05_GBO=mean(objectiveData_all_server_N05,2,'omitnan');
mean_objectiveData_all_server_N010_GBO=mean(objectiveData_all_server_N010,2,'omitnan');
mean_objectiveData_all_server_N020_GBO=mean(objectiveData_all_server_N020,2,'omitnan');
% =========================================================================

true_objective=0.1882; %with ts=1
f2=figure(2);hold on
f2.Position=[200 0 1000 600];
hmean_N03=semilogy(mean_objectiveData_all_server_N03./true_objective, '--', 'LineWidth', 1, 'DisplayName','BO');
hmean_N05=semilogy(mean_objectiveData_all_server_N05./true_objective, '--', 'LineWidth', 1, 'DisplayName','BO');
hmean_N010=semilogy(mean_objectiveData_all_server_N010./true_objective, '--', 'LineWidth', 1, 'DisplayName','BO');
hmean_N020=semilogy(mean_objectiveData_all_server_N020./true_objective, '--', 'LineWidth', 1, 'DisplayName','BO');

true_objective=0.1243; %with ts=0.1
hmean_N03_GBO=semilogy(mean_objectiveData_all_server_N03_GBO./true_objective, 'LineWidth', 1, 'DisplayName','GBO');
hmean_N05_GBO=semilogy(mean_objectiveData_all_server_N05_GBO./true_objective, 'LineWidth', 1, 'DisplayName','GBO');
hmean_N010_GBO=semilogy(mean_objectiveData_all_server_N010_GBO./true_objective, 'LineWidth', 1, 'DisplayName','GBO');
hmean_N020_GBO=semilogy(mean_objectiveData_all_server_N020_GBO./true_objective, 'LineWidth', 1, 'DisplayName','GBO');
% hmean_surrogate=semilogy(mean_objectiveData_all_server_surrogate./true_objective, 'r', 'LineWidth', 4, 'DisplayName','BO');
legend([hmean_N03, hmean_N05, hmean_N010, hmean_N020, hmean_N03_GBO, hmean_N05_GBO, hmean_N010_GBO, hmean_N020_GBO],{'BO, N0=3','BO, N0=5', 'BO, N0=10', 'BO, N0=20', 'GBO, N0=3','GBO, N0=5', 'GBO, N0=10', 'GBO, N0=20'}, 'Location', 'best')
grid on
% ylim([0.1 0.3])
ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Minimum Observed Objective vs Iterations over Real Plant for BO and Guided BO'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
% figName=append(dir, 'objectiveData_all.png');
figName=append(dir, 'server_', 'ALL_means','.png');
saveas(gcf,figName)
% figName=append(dir, 'objectiveData_all.fig');
figName=append(dir, 'server_', 'ALL_means','.fig');
saveas(gcf,figName)

pause;
close all;

end