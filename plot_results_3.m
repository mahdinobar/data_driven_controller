function plot_results_3
% hyper-params
idName= 'Histogram_N01';
N0=1;
N_iter=50;
repeat_experiment=250;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

% dir=append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', idName, '/');
dir='/home/mahdi/PhD application/ETH/Rupenyan/code/results/server_runs_ver_3_2';
% load(append(dir,'InitData_all'))
% % InitData_all=table2array(InitData_all);
% load(append(dir,'InitobjectiveData_all.mat'))
% % load(append(dir,'objectiveData_all.mat'))
% load(append(dir,'objectiveEstData_all.mat'))
% objectiveData_all=InitobjectiveData_all;

% load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_20_6/InitobjectiveData_all.mat')
% objectiveData_all_server_1=InitobjectiveData_all;
% objectiveData_all_server_1=reshape(objectiveData_all_server_1(N0+1:end),[N_iter-25,repeat_experiment]);

tmp=[];
for m=33:36
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_1_tmp=InitobjectiveData_all;
%     objectiveData_all_server_1_tmp=objectiveData_all;
    objectiveData_all_server_1_tmp=reshape(objectiveData_all_server_1_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_1_tmp];
end
% tmp(tmp<0) = NaN;
objectiveData_all_server_1=tmp;

tmp=[];
for m=37:40
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
%     load(append(dir,'/objectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_surrogate_tmp=InitobjectiveData_all;
%     objectiveData_all_server_surrogate_tmp=objectiveData_all;
    objectiveData_all_server_surrogate_tmp=reshape(objectiveData_all_server_surrogate_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_surrogate_tmp];
end

objectiveData_all_server_surrogate=tmp;

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
f2.Position=[200 0 1600 800];

% for i=1:repeat_experiment
%     for j=1:N_iter-10
%         objectiveData_all(j,i)=min(objectiveData_all(1:j,i));
%     end
% end

for i=1:1000
    for j=1:50
        objectiveData_all_server_1(j,i)=nanmin(objectiveData_all_server_1(1:j,i));
        objectiveData_all_server_surrogate(j,i)=nanmin(objectiveData_all_server_surrogate(1:j,i));
    end
end

% objectiveData_all(:,5)=objectiveData_all(:,4);
% true_objective=0.1882;
true_objective=0.1243; %with ts=0.1

edges = [1:.01:1.1];
h=histogram(objectiveData_all_server_1(end,:)/true_objective, edges, 'FaceColor', [0 0 1], 'Normalization','probability');
h_surrogate=histogram(objectiveData_all_server_surrogate(end,:)/true_objective, edges, 'FaceColor', [1 0 0], 'Normalization','probability');
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend([h_surrogate h],{'Guided BO','BO'}, 'Location', 'best')
grid on
% ylim([0.1 0.3])
% ylim([1 2])
xlabel('Optimality Ratio')
ylabel('Percentage')
title(append('Minimum Observed Objective Histogram after 50 Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
figName=append(dir, '_server_', idName,'.png');
saveas(gcf,figName)
figName=append(dir, '_server_', idName,'.fig');
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