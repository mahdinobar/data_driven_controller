function plot_results
% hyper-params
idName= 'demo_17_2';
N0=10;
N_iter=50;
repeat_experiment=100;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir=append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', idName, '/');
% load(append(dir,'InitData_all'))
% InitData_all=table2array(InitData_all);
load(append(dir,'objectiveData_all.mat'))
load(append(dir,'objectiveEstData_all.mat'))

objectiveEstData_all=reshape(objectiveEstData_all(N0+1:end),[N_iter,repeat_experiment]);
mean_objectiveEstData_all=mean(objectiveEstData_all,2);
objectiveData_all=reshape(objectiveData_all(N0+1:end),[N_iter,repeat_experiment]);
mean_objectiveData_all=mean(objectiveData_all,2);

CI=[];
CI_Est=[];
for i=1:N_iter-N0
    x = objectiveEstData_all(i,:);                      % Create Data
    SEM = std(x)/sqrt(length(x));               % Standard Error
    ts = tinv([0.025  0.975],length(x)-1);      % T-Score
    CI_Est = [CI_Est; mean(x) + ts*SEM];
    
    x = objectiveData_all(i,:);                      % Create Data
    SEM = std(x)/sqrt(length(x));               % Standard Error
    ts = tinv([0.025  0.975],length(x)-1);      % T-Score
    CI = [CI; mean(x) + ts*SEM];
end 

figure(1);hold on
for i=1:N_iter-N0
    semilogy(objectiveEstData_all(:,i), '-', 'Color', [0, 0, 1, 0.05], 'LineWidth', 0.1)
end
hCI=semilogy(CI_Est(:,1), ':r', 'LineWidth', 2, 'DisplayName','95% confidence interval');
semilogy(CI_Est(:,2), ':r', 'LineWidth', 2)
hmean=semilogy(mean_objectiveEstData_all, 'b', 'LineWidth', 3, 'DisplayName','mean');
legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
grid on
xlabel('Iteration')
ylabel('Estimated Model Objective')
figName=append(dir, 'objectiveEstData_all.png');
saveas(gcf,figName)
figName=append(dir, 'objectiveEstData_all.fig');
saveas(gcf,figName)

figure(2);hold on
for i=1:N_iter-N0
    semilogy(objectiveData_all(:,i), '-', 'Color', [0, 0, 1, 0.05], 'LineWidth', 0.1)
end
hCI = semilogy(CI(:,1), ':r', 'LineWidth', 2, 'DisplayName','95% confidence interval');
semilogy(CI(:,2), ':r', 'LineWidth', 2)
hmean = semilogy(mean_objectiveData_all, 'b', 'LineWidth', 3, 'DisplayName','mean');
legend([hCI hmean],{'95% confidence interval','mean'}, 'Location', 'best')
grid on
xlabel('Iteration')
ylabel('Model Objective')

% 
% % plot(objectiveEstData_all(end-:end), 'b','DisplayName','MinObjective')
% if withSurrogate==true
%     surrogate_iteration=1:N_surrogate_repeat+1:N_iter-N0;
%     for i = 1:size(surrogate_iteration,2)
%         xline(surrogate_iteration(i));
%     end
% end
% plot(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
% xlabel('iteration')
% ylabel('objective')
% ylim([-0.01 0.01])
% xlim([1 N_iter-N0])
% dir='/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_15/';
% figName=append(dir, idName, '.png');
% % saveas(gcf,figName)
% 
% figure(2);hold on
% semilogy(objectiveData_not_removed(N0+1:end), 'b','DisplayName','MinObjective')
% if withSurrogate==true
%     for i = 1:size(surrogate_iteration,2)
%         xline(surrogate_iteration(i));
%     end
% end
% semilogy(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
% legend('MinObjective','MinEstimatedObjective')
% xlabel('iteration')
% ylabel('objective')
% xlim([1 N_iter-N0])
% figName=append(dir, idName, '_log.png');
% % saveas(gcf,figName)
% 
% objectiveData_dir=append(dir, idName, '_objectiveData_not_removed.mat');
% % save(objectiveData_dir,'objectiveData_not_removed');
% objectiveEstData_dir=append(dir, idName, '_objectiveEstData_not_removed.mat');
% % save(objectiveEstData_dir,'objectiveEstData_not_removed');
% InitData_dir=append(dir, idName, '_InitData.mat');
% % save(InitData_dir,'InitData');
% 
pause;
close all;

end