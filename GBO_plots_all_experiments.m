% function GBO_plots_all_experiments(TraceGBO, N0, N_iter, idName)
% dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');

% =========================================================================
% uncomment for server plots
function GBO_plots_all_experiments
idName= 'results_1';
dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_3/', idName, '/');
N0=1; %number of initial data
N_iter=50;
N_iter=N_iter+N0;
% todo automatize code
load(append(dir,'trace_file.mat'),'Trace')
TraceGBO=Trace;
clear Trace
% =========================================================================

% todo automatize code
load(append(dir,'trace_file_BO.mat'),'Trace')
TraceBO=Trace;
clear Trace

% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 4.1000;
ms_true=[0.6119, 1.6642];

fig=figure();
hold on
fig.Position=[200 0 1600 1200];
for expr=1:length(TraceGBO)
    JminObservGBO(:,expr)=TraceGBO(expr).values(N0+1:N_iter);
    JminObservBO(:,expr)=TraceBO(expr).values(N0+1:N_iter);
    for j=1+N0:N_iter
        JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
        JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
    end
    h1=semilogy(JminObservGBO(:,expr)./true_objective, ':', 'Color', [1, 0, 0, .5], 'LineWidth', .5);
    h2=semilogy(JminObservBO(:,expr)./true_objective, ':', 'Color', [0, 0, 1, .5], 'LineWidth', .5);
end

meanJminObservGBO=nanmean(JminObservGBO,2);
h3=semilogy(meanJminObservGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 4);

meanJminObservBO=nanmean(JminObservBO,2);
h4=semilogy(meanJminObservBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 4);

legend([h1, h2, h3, h4],{'GBO: Minimum Observed Evaluation', 'BO: Minimum Observed Evaluation', 'GBO: Monte Carlo Mean', 'BO: Monte Carlo Mean'}, 'Location', 'best');
grid on
ylim([1 10])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_ORi_MonteCarlo.png');
saveas(gcf,figName)
figName=append(dir, idName,'_ORi_MonteCarlo.fig');
saveas(gcf,figName)
end