function GBO_plots_all_experiments()%(ms, mv, TraceGBO, gain_mins, gain_maxes, N0, N_iter, N_G, idName, G)
clc
clear all;
% hyper-params
idName= 'demo_GBO_1_7';
sys='DC_motor';
N0=5;
N_expr=2;
N_iter=30;
N_iter=N_iter+N0;
Nsample=50;
withSurrogate=false;
if withSurrogate
    npG2=2;
    N_G = 5; %number of consecutive optimization on real plant before surrogate
    N_extra= 6; % to compensate deleted iteration of surrogate
    N_iter=N_iter+N_extra;
end



dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');
load(append(dir,'trace_file.mat'))
TraceGBO=Trace;
clear Trace
% todo automatize code
load(append(dir,'trace_file_BO.mat'))
TraceBO=Trace;
clear Trace

% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 2.43936437324367;
ms_true=[0.6119, 1.6642];

fig=figure();hold on
fig.Position=[200 0 1600 1200];
for expr=1:length(TraceGBO)
    JminObservGBO(:,expr)=TraceGBO(expr).values(N0+1:end);
    JminObservBO(:,expr)=TraceBO(expr).values(N0+1:end);
    for j=1+N0:N_iter
        JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
        JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
    end
    h1=semilogy(JminObservGBO(:,expr)./true_objective, ':', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
    h2=semilogy(JminObservBO(:,expr)./true_objective, ':', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
end

meanJminObservGBO=nanmean(JminObservGBO,2);
h3=semilogy(meanJminObservGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 3);

meanJminObservBO=nanmean(JminObservBO,2);
h4=semilogy(meanJminObservBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3);

legend([h1, h2, h3, h4],{'GBO: Minimum Observed Evaluation', 'BO: Minimum Observed Evaluation', 'GBO: Monte Carlo Mean', 'BO: Monte Carlo Mean'}, 'Location', 'best');
grid on
% ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
% set(gca, 'DefaultAxesFontName', 'Times')
% set(gca,'yscale','log')
% figName=append(dir, idName,'_ORi_MonteCarlo.png');
% saveas(gcf,figName)
% figName=append(dir, idName,'_ORi_MonteCarlo.fig');
% saveas(gcf,figName)
end

% function draft()
% close all; clear;
% load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_1_1/trace_file_BO.mat')
% TraceBO=Trace;
% clear Trace
% load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_1_1/trace_file.mat')
% TraceGBO=Trace;
% clear Trace
% 
% N_iter=33;
% N0=3;
% true_objective = 2.43936437324367;
% 
% fig=figure();hold on
% fig.Position=[200 0 1600 800];
% grid on
% % c = linspace(1,N_iter-N0,N_iter-N0);
% % h=scatter(TraceGBO.samples(N0+1:end, 1), TraceGBO.values(N0+1:end)./true_objective,[],c,'filled', 'SizeData', 100);
% % h2=scatter(TraceBO.samples(N0+1:end, 1), TraceBO.values(N0+1:end)./true_objective,[],c, 'filled', '^', 'SizeData', 100);
% 
% h3=plot(TraceGBO.post_mus(N0+1:end), 'Color', [0, 1, 0, 1], 'LineWidth', 3);
% y=TraceGBO.post_mus(N0+1:end);
% CI=TraceGBO.post_sigma2s(N0+1:end)/2;
% x=linspace(1,length(y),length(y))';
% 
% plot(y-CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1)
% plot(y+CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1)
% 
% x=0:5:29;
% y=TraceGBO.G2_post_mus;
% err=TraceGBO.G2_post_sigma2s;
% errorbar(x, y, err, '-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red', 'LineStyle','none');
% 
% 
% % fill([x;fliplr(x)], [y-CI;fliplr(y+CI)], [0, 1, 0], 'FaceAlpha',0.5)
% 
% % cbar = colorbar;
% % colormap winter
% % ylabel(cbar, 'iteration')
% % legend([h, h2],{'GBO', 'BO'}, 'Location', 'best');
% % grid on
% % xlabel('Kp')
% % ylabel('Optimality Ratio')
% % xlim([gain_mins(1), gain_maxes(1)])
% % % ylabel('Cost function')
% % title(append('Objective vs Kp gain (N0=',num2str(N0),')'))
% 
% end
