function plot_results
close all; clear; clc;
% hyper-params
idName= 'N03_GBO_variousSurrogatevsReal';
sys='robot_arm';
N0=3;
N_iter=50+25;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=10;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

dir='//home/mahdi/ETH/GBO/code/results/server_runs_ver_3';
dir_2='//home/mahdi/ETH/GBO/code/results/server_runs_ver_4';
dir_3='//home/mahdi/ETH/GBO/code/results/server_runs_ver_5';
dir_4='//home/mahdi/ETH/GBO/code/results/server_runs_ver_5';
dir_5='//home/mahdi/ETH/GBO/code/results/server_runs_ver_5';
dir_6='//home/mahdi/ETH/GBO/code/results/server_runs_ver_5';

tmp=[];
for m=1:4
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_0_tmp=InitobjectiveData_all;
    objectiveData_all_server_0_tmp=reshape(objectiveData_all_server_0_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_0_tmp];
end
objectiveData_all_server_0=tmp;

tmp=[];
for m=17:20
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_1_tmp=InitobjectiveData_all;
    objectiveData_all_server_1_tmp=reshape(objectiveData_all_server_1_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_1_tmp];
end
objectiveData_all_server_1=tmp;

tmp=[];
for m=1:4
    load(append(dir_2,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_2_tmp=InitobjectiveData_all;
    objectiveData_all_server_2_tmp=reshape(objectiveData_all_server_2_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_2_tmp];
end
objectiveData_all_server_2=tmp;


tmp=[];
for m=1:4
    load(append(dir_3,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_3_tmp=InitobjectiveData_all;
    objectiveData_all_server_3_tmp=reshape(objectiveData_all_server_3_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_3_tmp];
end
objectiveData_all_server_3=tmp;

tmp=[];
for m=5:8
    load(append(dir_4,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_4_tmp=InitobjectiveData_all;
    objectiveData_all_server_4_tmp=reshape(objectiveData_all_server_4_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_4_tmp];
end
objectiveData_all_server_4=tmp;

tmp=[];
for m=9:12
    load(append(dir_5,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_5_tmp=InitobjectiveData_all;
    objectiveData_all_server_5_tmp=reshape(objectiveData_all_server_5_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_5_tmp];
end
objectiveData_all_server_5=tmp;

% tmp=[];
% for m=13:16
%     load(append(dir_6,'/InitobjectiveData_all_',num2str(m),'.mat'))
%     objectiveData_all_server_6_tmp=InitobjectiveData_all;
%     objectiveData_all_server_6_tmp=reshape(objectiveData_all_server_6_tmp(1:end),[50,250]);
%     tmp=[tmp, objectiveData_all_server_6_tmp];
% end
% objectiveData_all_server_6=tmp;

f2=figure(2);hold on
f2.Position=[200 0 1600 800];


for i=1:1000
    for j=1:50
        objectiveData_all_server_0(j,i)=nanmin(objectiveData_all_server_0(1:j,i));
        objectiveData_all_server_1(j,i)=nanmin(objectiveData_all_server_1(1:j,i));
        objectiveData_all_server_2(j,i)=nanmin(objectiveData_all_server_2(1:j,i));
        objectiveData_all_server_3(j,i)=nanmin(objectiveData_all_server_3(1:j,i));
        objectiveData_all_server_4(j,i)=nanmin(objectiveData_all_server_4(1:j,i));
        objectiveData_all_server_5(j,i)=nanmin(objectiveData_all_server_5(1:j,i));
%         objectiveData_all_server_6(j,i)=nanmin(objectiveData_all_server_6(1:j,i));
    end
end

% true_objective=0.1882; %with ts=1
true_objective=0.1243; %with ts=0.1

% for i=1:1000
%     semilogy(objectiveData_all_server_0(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0, 0, 1, .4], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_1(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [1, 0, 0, .4], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_2(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0, 1, 0, .4], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_3(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0, 1, 1, .4], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_4(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0.4940, 0.1840, 0.5560, .4], 'LineWidth', 0.1)
%     semilogy(objectiveData_all_server_5(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0.9290, 0.6940, 0.1250, .4], 'LineWidth', 0.1)
% %     semilogy(objectiveData_all_server_6(:,i)./true_objective, ':', 'LineWidth', .1, 'Color', [0.8500, 0.3250, 0.0980, .4], 'LineWidth', 0.1)
% end

mean_objectiveData_all_server_0=mean(objectiveData_all_server_0,2,'omitnan');
mean_objectiveData_all_server_1=mean(objectiveData_all_server_1,2,'omitnan');
mean_objectiveData_all_server_2=mean(objectiveData_all_server_2,2,'omitnan');
mean_objectiveData_all_server_3=mean(objectiveData_all_server_3,2,'omitnan');
mean_objectiveData_all_server_4=mean(objectiveData_all_server_4,2,'omitnan');
mean_objectiveData_all_server_5=mean(objectiveData_all_server_5,2,'omitnan');
% mean_objectiveData_all_server_6=mean(objectiveData_all_server_6,2,'omitnan');

hmean_0=semilogy(mean_objectiveData_all_server_0./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 2, 'DisplayName','BO');
hmean_1=semilogy(mean_objectiveData_all_server_1./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 2, 'DisplayName','BO');
hmean_2=semilogy(mean_objectiveData_all_server_2./true_objective, 'Color', [0, .7, 0, 1], 'LineWidth', 2, 'DisplayName','BO');
hmean_3=semilogy(mean_objectiveData_all_server_3./true_objective, 'Color', [0, .9, .9, 1], 'LineWidth', 2, 'DisplayName','BO');
hmean_4=semilogy(mean_objectiveData_all_server_4./true_objective, 'Color', [0.4940, 0.1840, 0.5560, 1], 'LineWidth', 2, 'DisplayName','BO');
hmean_5=semilogy(mean_objectiveData_all_server_5./true_objective, 'Color', [0.9290, 0.6940, 0.1250, 1], 'LineWidth', 2, 'DisplayName','BO');
% hmean_6=semilogy(mean_objectiveData_all_server_6./true_objective, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 5, 'DisplayName','BO');
legend([hmean_0, hmean_1, hmean_2, hmean_3, hmean_4, hmean_5],{'BO','GBO: 5 identified per 25 real plant','GBO: 5 identified per 10 real plant','GBO: 5 identified per 5 real plant','GBO: 1 identified per 3 real plant','GBO: 1 identified per 2 real plant'}, 'Location', 'best')
grid on
ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Minimum Observed Objective vs Iterations over Real Plant (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, '_server_', idName,'.png');
saveas(gcf,figName)
figName=append(dir, '_server_', idName,'.fig');
saveas(gcf,figName)

pause;
close all;

end