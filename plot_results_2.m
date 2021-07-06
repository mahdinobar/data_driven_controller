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

% dir=append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', idName, '/');
dir='/home/mahdi/PhD application/ETH/Rupenyan/code/results/server_runs_ver_3';
% load(append(dir,'InitData_all'))
% % InitData_all=table2array(InitData_all);
% load(append(dir,'InitobjectiveData_all.mat'))
% % load(append(dir,'objectiveData_all.mat'))
% load(append(dir,'objectiveEstData_all.mat'))
% objectiveData_all=InitobjectiveData_all;

% load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_20_6/InitobjectiveData_all.mat')
% objectiveData_all_server_1=InitobjectiveData_all;
% objectiveData_all_server_1=reshape(objectiveData_all_server_1(N0+1:end),[N_iter-25,repeat_experiment]);

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
% tmp(tmp<0) = NaN;
objectiveData_all_server_N03=tmp_N03;
objectiveData_all_server_N05=tmp_N05;
objectiveData_all_server_N010=tmp_N010;
objectiveData_all_server_N020=tmp_N020;

% tmp_N03=[];
% for m=17:20
%     load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
% %     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
%     objectiveData_all_server_surrogate_tmp=InitobjectiveData_all;
% %     objectiveData_all_server_surrogate_tmp=objectiveData_all;
%     objectiveData_all_server_surrogate_tmp=reshape(objectiveData_all_server_surrogate_tmp(1:end),[50,250]);
%     tmp_N03=[tmp_N03, objectiveData_all_server_surrogate_tmp];
% end
% 
% objectiveData_all_server_surrogate=tmp_N03;

% objectiveEstData_all=reshape(objectiveEstData_all(N0+1:end),[N_iter-10,repeat_experiment]);

% TF = abs(objectiveEstData_all)<1e3;
% % TF = isoutlier(objectiveEstData_all, 2);
% objectiveEstData_all=objectiveEstData_all.*TF;
% % TF2=isoutlier(objectiveEstData_all, 2);
% % objectiveEstData_all=objectiveEstData_all.*TF2;
% objectiveEstData_all(objectiveEstData_all == 0) = NaN;

% objectiveData_all=reshape(objectiveData_all(N0+1:end),[N_iter-10,repeat_experiment]);

% objectiveData_all=objectiveData_all.*TF;
% objectiveData_all(objectiveData_all == 0) = NaN;


% f=figure(1);hold on
% f.Position=[0 0 1000 600];
% for i=1:N_iter-N0
%     x=linspace(1,repeat_experiment,repeat_experiment);
%     nan_flag=isnan(objectiveData_all(i,:));
%     semilogy(x(~nan_flag),objectiveEstData_all(i,~nan_flag), '-', 'Color', [0, 0, 1, 1], 'LineWidth', 0.1)
% end
% hCI=semilogy(CI_Est(:,1), '--r', 'LineWidth', 1, 'DisplayName','95% confidence interval');
% semilogy(CI_Est(:,2), '--r', 'LineWidth', 1)
% hmean=semilogy(mean_objectiveEstData_all, 'k', 'LineWidth', 1.5, 'DisplayName','mean');
% legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
% grid on
% % ylim([70 90])
% xlabel('Iteration')
% ylabel('Estimated Model Objective')
% title('Bayesian Optimization Estimated Model Objective vs Iterations over Real Plant')
% figName=append(dir, 'objectiveEstData_all.png');
% saveas(gcf,figName)
% figName=append(dir, 'objectiveEstData_all.fig');
% saveas(gcf,figName)

f2=figure(2);hold on
f2.Position=[200 0 1000 600];
% for i=1:repeat_experiment
%     for j=1:N_iter-10
%         objectiveData_all(j,i)=min(objectiveData_all(1:j,i));
%     end
% end

for i=1:1000
    for j=1:50
        objectiveData_all_server_N03(j,i)=nanmin(objectiveData_all_server_N03(1:j,i));
        objectiveData_all_server_N05(j,i)=nanmin(objectiveData_all_server_N05(1:j,i));
        objectiveData_all_server_N010(j,i)=nanmin(objectiveData_all_server_N010(1:j,i));
        objectiveData_all_server_N020(j,i)=nanmin(objectiveData_all_server_N020(1:j,i));
%         objectiveData_all_server_surrogate(j,i)=nanmin(objectiveData_all_server_surrogate(1:j,i));
    end
end

% objectiveData_all(:,5)=objectiveData_all(:,4);
true_objective=0.1882;

% for i=1:repeat_experiment
%     semilogy(objectiveData_all(:,i)./true_objective, ':', 'LineWidth', 1, 'Color', [1, 0, 0, .2], 'LineWidth', 0.1)
% end

% for i=1:1000
%     semilogy(objectiveData_all_server_N03(:,i)./true_objective, ':', 'LineWidth', 1, 'Color', [0, 0, 1, .3], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_surrogate(:,i)./true_objective, ':', 'LineWidth', 1, 'Color', [1, 0, 0, .3], 'LineWidth', 0.1)
% end

% mean_objectiveData_all=mean(objectiveData_all(:,:),2,'omitnan');
mean_objectiveData_all_server_N03=mean(objectiveData_all_server_N03,2,'omitnan');
mean_objectiveData_all_server_N05=mean(objectiveData_all_server_N05,2,'omitnan');
mean_objectiveData_all_server_N010=mean(objectiveData_all_server_N010,2,'omitnan');
mean_objectiveData_all_server_N020=mean(objectiveData_all_server_N020,2,'omitnan');
% mean_objectiveData_all_server_surrogate=mean(objectiveData_all_server_surrogate,2,'omitnan');

% CI=[];
% CI_Est=[];
% for i=1:size(objectiveEstData_all,1)    
%     x = objectiveData_all(i,~isnan(objectiveData_all(i,:)));                      % Create Data
%     SEM = std(x)/sqrt(length(x));               % Standard Error
%     ts = tinv([0.025  0.975],length(x)-1);      % T-Score
%     CI = [CI; mean(x,'omitnan') + ts*SEM];
% end 

% hCI=semilogy(CI(:,1), '--r', 'LineWidth', 1, 'DisplayName','95% confidence interval');
% semilogy(CI(:,2), '--r', 'LineWidth', 1)
% hmean=semilogy(mean_objectiveData_all./true_objective, 'r', 'LineWidth', 4, 'DisplayName','Guided BO');
hmean_N03=semilogy(mean_objectiveData_all_server_N03./true_objective, 'LineWidth', 1, 'DisplayName','BO');
hmean_N05=semilogy(mean_objectiveData_all_server_N05./true_objective, 'LineWidth', 1, 'DisplayName','BO');
hmean_N010=semilogy(mean_objectiveData_all_server_N010./true_objective, 'LineWidth', 1, 'DisplayName','BO');
hmean_N020=semilogy(mean_objectiveData_all_server_N020./true_objective, 'LineWidth', 1, 'DisplayName','BO');
% hmean_surrogate=semilogy(mean_objectiveData_all_server_surrogate./true_objective, 'r', 'LineWidth', 4, 'DisplayName','BO');
legend([hmean_N03, hmean_N05, hmean_N010, hmean_N020],{'N0=3','N0=5', 'N0=10', 'N0=20'}, 'Location', 'best')
grid on
% ylim([0.1 0.3])
ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Minimum Observed Objective vs Iterations over Real Plant for Guided BO'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
% figName=append(dir, 'objectiveData_all.png');
figName=append(dir, 'server_', 'only_Guided_BO','.png');
saveas(gcf,figName)
% figName=append(dir, 'objectiveData_all.fig');
figName=append(dir, 'server_', 'only_Guided_BO','.fig');
saveas(gcf,figName)


% f3=figure(3);hold on
% f3.Position=[0 200 1000 600];
% for i=1:repeat_experiment
%     for j=1:N_iter-10
%         objectiveEstData_all(j,i)=min(objectiveEstData_all(1:j,i));
%     end
% end
% 
% % objectiveData_all(:,5)=objectiveData_all(:,4);
% 
% for i=1:repeat_experiment
%     semilogy(objectiveEstData_all(:,i), '-', 'Color', [0, 0, 1, .2], 'LineWidth', 0.1)
% end
% 
% mean_objectiveEstData_all=mean(objectiveEstData_all,2,'omitnan');
% 
% CI=[];
% CI_Est=[];
% for i=1:size(objectiveEstData_all,1)
%     x = objectiveEstData_all(i,~isnan(objectiveEstData_all(i,:)));                      % Create Data
%     SEM = std(x)/sqrt(length(x));               % Standard Error
%     ts = tinv([0.025  0.975],length(x)-1);      % T-Score
%     CI_Est = [CI_Est; mean(x,'omitnan') + ts*SEM];
%    
% end 
% 
% hCI=semilogy(CI_Est(:,1), '--r', 'LineWidth', 1, 'DisplayName','95% confidence interval');
% semilogy(CI_Est(:,2), '--r', 'LineWidth', 1)
% hmean=semilogy(mean_objectiveEstData_all, 'k', 'LineWidth', 1.5, 'DisplayName','mean');
% legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
% grid on
% ylim([0.1 0.3])
% xlabel('Iteration')
% ylabel('Observed Model Objective')
% title('Observed Objective vs Iterations over Real Plant')
% figName=append(dir, 'objectiveData_all.png');
% saveas(gcf,figName)
% figName=append(dir, 'objectiveData_all.fig');
% saveas(gcf,figName)

% f2=figure(3);hold on
% f2.Position=[0 0 1000 600];
% obj_exp=[];
% for j=1:repeat_experiment
%     for i=1:N_iter
%         obj_exp=[obj_exp;min(objectiveData_all(1:i,j),[],'all')];
%     end
% %     semilogy(objectiveData_all(:,i), '-', 'Color', [0, 0, 1, 0.5], 'LineWidth', 0.1)
% end
% obj_exp=reshape(obj_exp,[N_iter,repeat_experiment]);
% semilogy(obj_exp, 'b', 'LineWidth', .5, 'DisplayName','mean');
% % hCI = semilogy(CI(:,1), '--r', 'LineWidth', 1, 'DisplayName','95% confidence interval');
% % semilogy(CI(:,2), '--r', 'LineWidth', 1)
% obj=[];
% for k=1:50
% %     objectiveData_all
%     
%     obj=[obj;min(objectiveData_all(1:k,:),[],'all')];
%     
% end
% hmean = semilogy(obj, 'k', 'LineWidth', 1.5, 'DisplayName','mean');
% legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
% grid on
% xlabel('Iteration')
% ylabel('Model Objective')
pause;
close all;

end