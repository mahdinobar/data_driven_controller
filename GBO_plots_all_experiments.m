function GBO_plots_all_experiments(TraceGBO, N0, N_iter, idName)


dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');

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
    JminObservGBO=TraceGBO(expr).values(N0+1:end);
    JminObservBO=TraceBO(expr).values(N0+1:end);
    for j=N0+1:N_iter
        JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(N0+1:j));
        JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(N0+1:j));
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
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_ORi_MonteCarlo.png');
saveas(gcf,figName)
figName=append(dir, idName,'_ORi_MonteCarlo.fig');
saveas(gcf,figName)
end