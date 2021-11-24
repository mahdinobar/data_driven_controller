function plot_results_base
close all; clear; clc;
% hyper-params
idName= '0';
sys='robot_arm';
N0=1;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_0/';

tmp=[];
load(append(dir,'/InitobjectiveData_all.mat'))
objectiveData_all_0=InitobjectiveData_all(N0+1:end,1);
objectiveData_all_0=reshape(objectiveData_all_0(1:end),[N_iter,repeat_experiment]);

f2=figure(2);hold on
f2.Position=[200 0 1600 800];


for i=1:repeat_experiment
    for j=1:N_iter
        objectiveData_all_0(j,i)=nanmin(objectiveData_all_0(1:j,i));
    end
end

true_objective=0.1882; %with ts=1
% true_objective=0.1243; %with ts=0.1

for i=1:repeat_experiment
    semilogy(objectiveData_all_0(:,i)./true_objective, ':', 'LineWidth', 1, 'Color', [0, 0, 1, .5])
end

mean_objectiveData_all_0=mean(objectiveData_all_0,2,'omitnan');

hmean_0=semilogy(mean_objectiveData_all_0./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3, 'DisplayName','BO');

legend([hmean_0],{'BO'}, 'Location', 'best')
grid on
ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Minimum Observed Objective vs Iterations over Real Plant (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'.png');
saveas(gcf,figName)
figName=append(dir, idName,'.fig');
saveas(gcf,figName)

pause;
close all;

end