function plot_results
% hyper-params
idName= 'demo_20_1';
sys='robot_arm';
N0=20;
N_iter=50+10;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

dir=append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', idName, '/');
load(append(dir,'InitData_all'))
% InitData_all=table2array(InitData_all);
load(append(dir,'InitobjectiveData_all.mat'))
% load(append(dir,'objectiveData_all.mat'))
load(append(dir,'objectiveEstData_all.mat'))
objectiveData_all=InitobjectiveData_all;
objectiveEstData_all=reshape(objectiveEstData_all(N0+1:end),[N_iter-10,repeat_experiment]);

% TF = abs(objectiveEstData_all)<1e3;
% % TF = isoutlier(objectiveEstData_all, 2);
% objectiveEstData_all=objectiveEstData_all.*TF;
% % TF2=isoutlier(objectiveEstData_all, 2);
% % objectiveEstData_all=objectiveEstData_all.*TF2;
% objectiveEstData_all(objectiveEstData_all == 0) = NaN;

objectiveData_all=reshape(objectiveData_all(N0+1:end),[N_iter-10,repeat_experiment]);
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
for i=1:repeat_experiment
    for j=1:N_iter-10
        objectiveData_all(j,i)=min(objectiveData_all(1:j,i));
    end
end

% objectiveData_all(:,5)=objectiveData_all(:,4);

for i=1:repeat_experiment
    semilogy(objectiveData_all(:,i), '-', 'Color', [0, 0, 1, .2], 'LineWidth', 0.1)
end

mean_objectiveData_all=mean(objectiveData_all,2,'omitnan');

CI=[];
CI_Est=[];
for i=1:size(objectiveEstData_all,1)    
    x = objectiveData_all(i,~isnan(objectiveData_all(i,:)));                      % Create Data
    SEM = std(x)/sqrt(length(x));               % Standard Error
    ts = tinv([0.025  0.975],length(x)-1);      % T-Score
    CI = [CI; mean(x,'omitnan') + ts*SEM];
end 

hCI=semilogy(CI(:,1), '--r', 'LineWidth', 1, 'DisplayName','95% confidence interval');
semilogy(CI(:,2), '--r', 'LineWidth', 1)
hmean=semilogy(mean_objectiveData_all, 'k', 'LineWidth', 1.5, 'DisplayName','mean');
legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
grid on
ylim([0.8 1.2])
xlabel('Iteration')
ylabel('Observed Model Objective')
title('Observed Objective vs Iterations over Real Plant')
figName=append(dir, 'objectiveData_all.png');
saveas(gcf,figName)
figName=append(dir, 'objectiveData_all.fig');
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