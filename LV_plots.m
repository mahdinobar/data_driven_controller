close all;
clc;
clear;

N0=1; %number of initial data
N_iter=50;
N_expr=18;
true_objective = 1;

tmp_name="exper_72_6";
tmp_dir=append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/",tmp_name);

% % find failed experiments
nan_expr=[];
for i=1:N_iter
    load(append(tmp_dir,'/BO_',string(i),'/perf_Data_',string(i),'.mat'))
    nan_perf=sum(isnan(perf_Data(:,1:4)));
    if nan_perf>1
        nan_expr=[nan_expr,i]
    end
end

JminObsBO_All=[];
JminObsGBO_All=[];
number_s=[];
number_idx_G2=[];
for expr=1:N_expr
%     remove manually outlier batches
    if expr==16 || expr==15 || expr==5 || expr==9
        expr=10;
    end

    JminObsBO=[];
    JminObsGBO=[];
    if isempty(find(nan_expr==expr, 1))
        dirBO=append(tmp_dir,'/BO_', string(expr), '/');
        dirGBO=append(tmp_dir,'/GBO_sw1_v5_', string(expr), '/');

%         load(append(dirGBO,'trace_file_expr_',num2str(expr),'.mat'))
        load(append(dirGBO,'trace_file_removed.mat'))
        TraceGBO=Trace;
%         expr
%         size(TraceGBO.values)
        clearvars Trace

%         load(append(dirGBO,'/when_switch_s.mat'))
%         number_s(end+1)=size(when_switch_s,1);

        load(append(dirGBO,'/idx_G2.mat'))
        number_idx_G2(end+1)=size(idx_G2,1);

%         % manual correction(TODO: remove later)
%         load(append(dirGBO,'/idx_G2.mat'))
%         TraceGBO.samples(idx_G2,:)=[];
%         TraceGBO.values(idx_G2)=[];
%         TraceGBO.post_mus(idx_G2)=[];
%         TraceGBO.post_sigma2s(idx_G2)=[];
%         TraceGBO.times(idx_G2)=[];
%         save(append(dirGBO, 'trace_file_expr_manually_corrected_',num2str(expr),'.mat'),'TraceGBO')



%         load(append(dirBO,'trace_file_expr_',num2str(expr),'.mat'))
        load(append(dirBO,'trace_file_removed.mat'))
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
noise_level=0.0439;
h1=semilogy(ax1, JminObsGBO_All(:,:)'./true_objective, ':', 'Color', [1, 0, 0, .5], 'LineWidth', 1.5);
h2=semilogy(ax1, JminObsBO_All(:,:)'./true_objective, ':', 'Color', [0, 0, 1, .5], 'LineWidth', 1.5);
h3=semilogy(ax1, meanJminObsGBO(:)./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 5);
h4=semilogy(ax1, meanJminObsBO(:)./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 5);
h5=semilogy(ax1, [meanJminObsGBO(:)./true_objective+noise_level,meanJminObsGBO(:)./true_objective-noise_level], "--", 'Color', [1, 0, 0, 1], 'LineWidth', 5);
h6=semilogy(ax1, [meanJminObsBO(:)./true_objective+noise_level,meanJminObsBO(:)./true_objective-noise_level], "--", 'Color', [0, 0, 1, 1], 'LineWidth', 5);
[a,b]=max(meanJminObsGBO<0.9915);
xlabel(ax1, 'Iteration on real plant')
ylabel(ax1, 'Minimum observed objective')
grid on
grid minor
% ylim([0 3])
xlim([1, 50])
xticks([1, 5:5:50])
legend([h3, h4, h5(1), h6(1)],{'GBO','BO','GBO noise level','BO noise level'}, 'Location', 'northeast');
figName=append(dirBO,'_experiments.png');
saveas(gcf,figName)
figName=append(dirBO,'_experiments.fig');
saveas(gcf,figName)
