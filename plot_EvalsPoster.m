function plot_EvalsPoster(TraceGBO, TraceBO, N0, N_iter, N_G, dir, idName)
%##########################################################################
% plot mean of sampled gains over all experiments
TraceGBOvalues=[];
TraceBOvalues=[];
JminObservGBO=[];
TraceGBOpost_mus=[];
TraceGBOG2_post_mus=[];
TraceBOpost_mus=[];
TraceGBOpost_sigma2s=[];
TraceGBOG2_post_sigma2s=[];
TraceBOpost_sigma2s=[];
for i=1:691
    TraceGBOvalues(:,i)=TraceGBO(i).values(1:end, 1);
    TraceBOvalues(:,i)=TraceBO(i).values(1:end, 1);
    TraceGBOpost_mus(:,i)=TraceGBO(i).post_mus(1:end, 1);
    TraceGBOpost_sigma2s(:,i)=TraceGBO(i).post_sigma2s(1:end, 1);
    TraceGBOG2_post_mus(:,i)=TraceGBO(i).G2_post_mus(1:end, 1);
    TraceGBOG2_post_sigma2s(:,i)=TraceGBO(i).G2_post_sigma2s(1:end, 1);
    TraceBOpost_mus(:,i)=TraceBO(i).post_mus(1:end, 1);
    TraceBOpost_sigma2s(:,i)=TraceBO(i).post_sigma2s(1:end, 1);
    for j=N0+1:N_iter
        JminObservGBO(j-N0,i)=nanmin(TraceGBO(i).values(N0+1:j));
        JminObservBO(j-N0,i)=nanmin(TraceBO(i).values(N0+1:j));
    end
end
TraceGBOvalues=mean(TraceGBOvalues,2);
TraceBOvalues=mean(TraceBOvalues,2);
JminObservGBO=mean(JminObservGBO,2);
TraceGBOpost_mus=mean(TraceGBOpost_mus,2);
TraceGBOpost_sigma2s=mean(TraceGBOpost_sigma2s,2);
TraceBOpost_mus=mean(TraceBOpost_mus,2);
TraceBOpost_sigma2s=mean(TraceBOpost_sigma2s,2);
TraceGBOG2_post_mus=mean(TraceGBOG2_post_mus,2);
TraceGBOG2_post_sigma2s=mean(TraceGBOG2_post_sigma2s,2);
JminObservBO=mean(JminObservBO,2);

fig=figure();hold on
fig.Position=[200 0 1600 800];
h1=plot(TraceGBOvalues(N0+1:end), '--', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
h2=plot(JminObservGBO, 'Color', [1, 0, 0, 1], 'LineWidth', 3);
% JminEst=TraceGBO.values(N0+1:end);
% for j=N0+1:N_iter
%     JminEst(j-N0)=nanmin(TraceGBO.post_mus(N0+1:j));
% end
h5=plot(TraceGBOpost_mus(N0+1:end), '--', 'Color', [1, 0, 0, 1], 'LineWidth', 3);
y=TraceGBOpost_mus(N0+1:end);
CI=TraceGBOpost_sigma2s(N0+1:end)/2;
x=linspace(1,length(y),length(y))';
h6=plot(y-CI, ':', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
plot(y+CI, ':', 'Color', [1, 0, 0, 1], 'LineWidth', 1)

h7=plot(TraceBOpost_mus(N0+1:end), '--', 'Color', [0, 0, 1, 1], 'LineWidth', 3);
y=TraceBOpost_mus(N0+1:end);
CI=TraceBOpost_sigma2s(N0+1:end)/2;
x=linspace(1,length(y),length(y))';
h8=plot(y-CI, ':', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
plot(y+CI, ':', 'Color', [0, 0, 1, 1], 'LineWidth', 1)

x=0:N_G:N_iter-N0-1;
y=TraceGBOG2_post_mus;
err=TraceGBOG2_post_sigma2s;
h9=errorbar(x, y, err, '-s','MarkerSize',12, 'Color', 'k', 'MarkerEdgeColor','k', 'LineWidth',1, 'LineStyle','none');
% h6=semilogy(JminEst./true_objective, 'Color', [0, 1, 0, 1], 'LineWidth', 3);
%--------------------------------------------------------------------------
% todo automatize code
% load(append(dir,'trace_file_BO.mat'))
% TraceBO=Trace;
% clear Trace
h3=plot(TraceBOvalues(N0+1:end), '--', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
h4=plot(JminObservBO, 'Color', [0, 0, 1, 1], 'LineWidth', 3);
legend([h1, h2, h3, h4, h5, h6, h7, h8, h9],{'GBO: Minimum Evaluated', ...
    'GBO: Minimum Observed Evaluation', 'BO: Minimum Evaluated', ...
    'BO: Minimum Observed Evaluation', 'GBO: Prediction', ...
    'GBO: 95% Confidence Interval', 'BO: Prediction', ...
    'BO: 95% Confidence Interval', 'GBO: Simulation Prediction with Confidence Bar'}, 'Location', 'southeast');
%--------------------------------------------------------------------------
grid on
ylim([-300 300])
xlabel('Iteration')
ylabel('J')
% ylabel('Cost function')
title(append('Objective vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
figName=append(dir, idName,'_Ji.png');
saveas(gcf,figName)
figName=append(dir, idName,'_Ji.fig');
saveas(gcf,figName)
%##########################################################################
