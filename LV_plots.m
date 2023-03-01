function LV_plots
close all;
clc;
clear;

N0=1; %number of initial data
N_iter=50;
N_expr=50;
true_objective = 1;

tmp_name="exper_72";
tmp_dir=append("C:\mahdi\data_driven_controller\Data\",tmp_name);

% find failed experiments
nan_expr=[];
for i=1:N_expr
    load(append('C:\mahdi\data_driven_controller\Data\exper_72\BO_',string(i),'\perf_Data_1_',string(i),'.mat'))
    nan_perf=sum(isnan(perf_Data(:,1:4)));
    if nan_perf>1
        nan_expr=[nan_expr,i];
    end
end

JminObsBO_All=[];
JminObsGBO_All=[];
for expr=1:N_expr
    JminObsBO=[];
    JminObsGBO=[];
    if isempty(find(nan_expr==expr, 1))
        dirBO=append(tmp_dir,'\BO_', string(expr), '\');
        dirGBO=dirBO;
        
        load(append(dirGBO,'trace_file.mat'),'Trace')
        TraceGBO=Trace;
        clearvars Trace
        load(append(dirBO,'trace_file.mat'),'Trace')
        TraceBO=Trace;
        clearvars Trace
        for j=1:N_iter
            JminObsGBO(end+1)=nanmin(TraceGBO.values(1:j));
            JminObsBO(end+1)=nanmin(TraceBO.values(1:j));
        end
        JminObsBO_All(end+1,:)=JminObsBO;
        JminObsGBO_All(end+1,:)=JminObsGBO;
    end
end

meanJminObsGBO=nanmean(JminObsGBO_All(:,:),1);
meanJminObsBO=nanmean(JminObsBO_All(:,:),1);

fig=figure();
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on
% h1=semilogy(ax1, JminObsGBO_All'./true_objective, ':', 'Color', [1, 0, 0, .5], 'LineWidth', 1.5);
h2=semilogy(ax1, JminObsBO_All'/true_objective, ':', 'Color', [0, 0, 1, .5], 'LineWidth', 1.5);
% h3=semilogy(ax1, meanJminObsGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 5);
h4=semilogy(ax1, meanJminObsBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 5);
[a,b]=max(meanJminObsGBO<0.9915);
xlabel(ax1, 'Iteration on real plant')
ylabel(ax1, 'Minimum observed objective')
grid on
grid minor
% ylim([0 3])
xlim([1, 50])
xticks([1, 5:5:50])
legend([h4],{'BO'}, 'Location', 'northeast');
figName=append(dirBO,'_experiments.png');
saveas(gcf,figName)
figName=append(dirBO,'_experiments.fig');
saveas(gcf,figName)
end