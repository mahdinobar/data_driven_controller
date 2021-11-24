function plot_results_4
close all; clear; clc;

% hyper-params
idName= 'Histogram_convergence';
N0=1;
N_iter=50;

dir='/home/mahdi/ETH/GBO/code/results/server_runs_ver_3';

tmp=[];
for m=1:4
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_1_tmp=InitobjectiveData_all;
    objectiveData_all_server_1_tmp=reshape(objectiveData_all_server_1_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_1_tmp];
end
objectiveData_all_server_1=tmp;

tmp=[];
for m=17:20
    load(append(dir,'/InitobjectiveData_all_',num2str(m),'.mat'))
    objectiveData_all_server_surrogate_tmp=InitobjectiveData_all;
    objectiveData_all_server_surrogate_tmp=reshape(objectiveData_all_server_surrogate_tmp(1:end),[50,250]);
    tmp=[tmp, objectiveData_all_server_surrogate_tmp];
end

objectiveData_all_server_surrogate=tmp;

for i=1:1000
    for j=1:50
        objectiveData_all_server_1(j,i)=nanmin(objectiveData_all_server_1(1:j,i));
        objectiveData_all_server_surrogate(j,i)=nanmin(objectiveData_all_server_surrogate(1:j,i));
    end
end

true_objective=0.1243; %with ts=0.1
th_opt_ratio=1.2;
aa=objectiveData_all_server_1/true_objective;
aa=aa>th_opt_ratio;
bb=objectiveData_all_server_surrogate/true_objective;
bb=bb>th_opt_ratio;
[~, columns] = size(aa);
AA = zeros(1, columns);
BB = zeros(1, columns);
for col = 1 : size(aa, 2)
    if sum(aa(:, col))>0
        AA(1, col) = find(aa(:, col), 1, 'last')+1;
    end
    if sum(bb(:, col))>0
        BB(1, col) = find(bb(:, col), 1, 'last')+1;
    end
end

f2=figure(2);hold on
f2.Position=[200 0 1600 800];
edges = [0:5:N0+N_iter];
h=histogram(AA, edges, 'FaceColor', [0 0 1], 'Normalization','probability');
h_surrogate=histogram(BB, edges, 'FaceColor', [1 0 0], 'Normalization','probability');
ytix = get(gca, 'YTick');
set(gca, 'YTick',ytix, 'YTickLabel',ytix*100)
legend([h_surrogate h],{'Guided BO','BO'}, 'Location', 'best')
grid on
xlabel('Total Number of Data')
ylabel('Percentage')
title(append('Covergence Histogram to Optimality Ratio ', th_opt_ratio))
set(gca, 'DefaultAxesFontName', 'Times')
figName=append(dir, '_server_', idName,'.png');
saveas(gcf,figName)
figName=append(dir, '_server_', idName,'.fig');
saveas(gcf,figName)


pause;
close all;

end