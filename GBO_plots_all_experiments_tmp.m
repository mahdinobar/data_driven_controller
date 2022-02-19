% comment for server plots
% function GBO_plots_all_experiments(TraceGBO, N0, N_iter, idName)
% dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');

% =========================================================================
% uncomment for server plots
function GBO_plots_all_experiments_tmp
close all;
clc;
clear;
idName= 'results_1';
dir=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_21' ...
    '/'], idName, '/');
N0=1; %number of initial data
N_iter=50;
N_iter=N_iter+N0;
% todo automatize code
load(append(dir,'trace_file.mat'),'Trace')
TraceGBO=Trace;
clear Trace
% =========================================================================

%% define plant
% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

% todo automatize code
load(append(dir,'trace_file_BO.mat'),'Trace')
TraceBO=Trace;
clear Trace

% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 4.18000;
ms_true=[0.6119, 1.6642];

for expr=1:min(length(TraceGBO),length(TraceBO))
    JminObservGBO(:,expr)=TraceGBO(expr).values(N0+1:N_iter);
    JminObservBO(:,expr)=TraceBO(expr).values(N0+1:N_iter);
    JsampleGBO(:,expr)=TraceGBO(expr).values(N0+1:N_iter);
    JsampleBO(:,expr)=TraceBO(expr).values(N0+1:N_iter);
    JminObservBO(:,expr)=TraceBO(expr).values(N0+1:N_iter);
    for j=1+N0:N_iter
        [minObs_tmp,minObs_Idx_tmp]=nanmin(TraceGBO(expr).values(1:j));
%         remove outlie solutions
        if minObs_tmp<true_objective*0.1
            Jcorrect = ObjFun(TraceGBO(expr).samples(minObs_Idx_tmp,:), G);
            TraceGBO(expr).values(minObs_Idx_tmp)=Jcorrect;
            JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
            JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
        else
            JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
            JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
        end
    end
%     h1=semilogy(ax1, JminObservGBO(:,expr)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', .5);
%     h2=semilogy(ax1, JminObservBO(:,expr)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', .5);
end

load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_30/',idName,'/meanJminObservGBO_infer_minObserved.mat'))
meanJminObservGBO_infer=meanJminObservGBO;

meanJminObservGBO=nanmean(JminObservGBO,2);
meanJminObservBO=nanmean(JminObservBO,2);



JsampleGBO(JsampleGBO>100)=100;
% meanJminObservGBO=nanmean(JsampleGBO,2);
% meanJminObservBO=nanmean(JsampleBO,2);
% save(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_30/',idName,'/meanJminObservGBO_infer.mat'),"meanJminObservGBO")

fig=figure();
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on
% h1=semilogy(ax1, JminObservGBO(:,:)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', .5);
% h2=semilogy(ax1, JminObservBO(:,:)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', .5);
h1=semilogy(ax1, meanJminObservGBO_infer./true_objective, 'Color', [0.9290 0.6940 0.1250 1], 'LineWidth', 3);

h3=semilogy(ax1, meanJminObservGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 3);

h4=semilogy(ax1, meanJminObservBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3);
h5=yline(1.39,  '--k', 'LineWidth', 3);
legend([h1, h3, h4, h5],{'Guided BO with inferior surrogate','Guided BO with superior surrogate','BO', 'Nominal Controller Threshold'}, 'Location', 'best');
grid on
ylim(ax1, [1 2])
xlim(ax1, [1 50])
yticks([1, 1.2, 1.4, 1.6, 1.8, 2])

fprintf('length(TraceGBO)=%d \n',length(TraceGBO))
fprintf('length(TraceBO)=%d \n',length(TraceBO))
thr=1.4;
[~,idx]=max(meanJminObservGBO./true_objective<thr);
fprintf('idx_GBO=%d \n',idx)
[~,idx]=max(meanJminObservBO./true_objective<thr);
fprintf('idx_BO=%d \n',idx)


% plot(p, '--p', 'LineWidth', 5)
% plot(ci(1,:)+meanJminObservBO', '--k', 'LineWidth', 5)
% plot(ax1, 1.2.*ones(size(meanJminObservGBO)),'--k', 'LineWidth',2)
% plot(ax1, 1.4.*ones(size(meanJminObservGBO)),'--k', 'LineWidth',2)

% addpath /home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Violinplot-Matlab-master/
% fig=figure();
% boxplot(x, 'PlotStyle','compact', 'OutlierSize', 2)
% vs = violinplot(x,0.5:49.5);




% group = [    .8:49.8;
%          1.2:50.2];
% 
% boxplot(x,.8:49.8, 'PlotStyle','compact', 'OutlierSize', 2, 'Colors','r')
% hold on
% boxplot(y,1.2:50.2, 'PlotStyle','compact', 'OutlierSize', 2, 'Colors','b')
% boxplot(x-y, 'PlotStyle','compact', 'OutlierSize', 2)
% ylim([1 3])
% set(gca,'yscale','log')

% boxplot([x,y], 'Notch','on', 'Labels',{1:50,1:50}, 'PlotStyle','compact', 'OutlierSize', 2)
% ylim([1 3])
% set(gca,'yscale','log')

xlabel(ax1, 'Iteration')
ylabel(ax1, 'Optimal Ratio')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
set(gca,'yscale','log')
figName=append(dir, idName,'MonteCarloEval.png');
saveas(gcf,figName)
figName=append(dir, idName,'MonteCarloEval.fig');
saveas(gcf,figName)
end


function [objective, constraints] = ObjFun(X, G)
%     todo move some lines outside with handler@: faster?
C=tf([X(1), X(1)*X(2)], [1, 0]);
CL=feedback(C*G, 1);

STPinfo=stepinfo(CL,'RiseTimeLimits',[0.1,0.6]);
ov=abs(STPinfo.Overshoot);
st=STPinfo.SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=STPinfo.RiseTime;
ITAE = trapz(t, t.*abs(e));

if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end

if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=1e5;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=1e5;
end

w=[0.1, 1, 1, 0.5];
w=w./sum(w);
objective=ov/w(1)+st/w(2)+Tr/w(3)+ITAE/w(4);
constraints=-1;
end