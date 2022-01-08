% % comment for server plots
% function GBO_plots_all_experiments(TraceGBO, N0, N_iter, idName, G2rmse)
% dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');


% =========================================================================
% uncomment for server plots
function GBO_plots_all_experiments
close all;
clc;
clear;
% idName= 'results_1';
% dir=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_21c2' ...
%     '/'], idName, '/');
idName= 'demo_GBO_1_11';
dir=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Expereiments_BEST_1/BO_Data' ...
    '/'], idName, '/');
idNameBO= 'demo_BO_1_11';
dirBO=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Expereiments_BEST_1/BO_Data' ...
    '/'], idNameBO, '/');
N0=1; %number of initial data
N_iter=50;
N_iter=N_iter+N0;
% todo automatize code
load(append(dir,'trace_file.mat'),'Trace')
TraceGBO=Trace;
TraceGBO.values(49)=[];
TraceGBO.samples(49, :)=[];
TraceGBO.post_mus(49)=[];
TraceGBO.post_sigma2s(49)=[];
clear Trace
% todo automatize code
load(append(dirBO,'trace_file.mat'),'Trace')
TraceBO=Trace;
clear Trace
% load(append(dir, 'G2rmse.mat'),'G2rmse')
% =========================================================================

% %% define plant
% % DC motor at FHNW lab
% num = [5.19908];
% den = [1, 1.61335];
% Td=2e-3;
% % MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
% G = tf(num, den, 'InputDelay',Td);

% % remove experiments with large rmse and OFF G2
% a=max(G2rmse(2:end,:),[],1)<0.5;
% TraceGBO=TraceGBO(find(a==1));
% TraceBO=TraceBO(find(a==1));
% sum(a)

% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 1;%4.1000;
% ms_true=[0.6119, 1.6642];
% true_objective=65.9974;
% true_objective=17.8676;
% true_objective=15.800;
expr=1;
while expr<min([length(TraceGBO),length(TraceBO)])+1
%     try    
    JminObservGBO(:,expr)=TraceGBO(expr).values(N0+1:N_iter);
    JminObservGBO_samples(:,expr,:)=TraceGBO(expr).samples(N0+1:N_iter,:);
    JminObservBO(:,expr)=TraceBO(expr).values(N0+1:N_iter);
    JminObservBO_samples(:,expr,:)=TraceBO(expr).samples(N0+1:N_iter,:);
%     catch
%         if expr==1
%             expr_tmp=expr+1;
%         else
%             expr_tmp=expr-1;
%         end
%         TraceGBO(expr)=TraceGBO(expr_tmp);
%         TraceBO(expr)=TraceBO(expr_tmp);
%         continue
%     end
    for j=1+N0:N_iter
        [minObs_tmp,minObs_Idx_tmp]=nanmin(TraceGBO(expr).values(1:j));
%         correct or remove outlie solutions because of computational
%         failure on server
        if minObs_tmp<true_objective-inf
            j
            Jcorrect = ObjFun(TraceGBO(expr).samples(minObs_Idx_tmp,:), G)
            TraceGBO(expr).values(minObs_Idx_tmp)=Jcorrect;
            JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
            JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
        else
            [JminObservGBO(j-N0,expr), NDX_GBO]=nanmin(TraceGBO(expr).values(1:j));
            JminObservGBO_samples(j-N0,expr,:)=TraceGBO(expr).samples(NDX_GBO,:);
            [JminObservBO(j-N0,expr), NDX_BO]=nanmin(TraceBO(expr).values(1:j));
            JminObservBO_samples(j-N0,expr,:)=TraceBO(expr).samples(NDX_BO,:);
        end
%         JminObservGBO(j-N0,expr)=nanmin(TraceGBO(expr).values(1:j));
%         JminObservBO(j-N0,expr)=nanmin(TraceBO(expr).values(1:j));
    end
%     h1=semilogy(ax1, JminObservGBO(:,expr)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', .5);
%     h2=semilogy(ax1, JminObservBO(:,expr)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', .5);
expr=expr+1;
end
meanJminObservGBO=nanmean(JminObservGBO,2);
meanJminObservBO=nanmean(JminObservBO,2);

% %%
% % uncomment to find gains corrosponding to the mean cost per iteration over all experiments
% [~,idxGBO]=min(abs(JminObservGBO-meanJminObservGBO),[],2);
% iteration_number=10;
% Gains_meanJ_GBO=JminObservGBO_samples(iteration_number,idxGBO(iteration_number),:)
% [~,idxBO]=min(abs(JminObservBO-meanJminObservBO),[],2);
% iteration_number=10;
% Gains_meanJ_BO=JminObservBO_samples(iteration_number,idxBO(iteration_number),:)

% WRONG calculation of meanJminObservGBO_samples so not use two lines below
% meanJminObservGBO_samples=squeeze(nanmean(JminObservGBO_samples,2));
% meanJminObservBO_samples=squeeze(nanmean(JminObservBO_samples,2));
%%


% x=JminObservGBO'./true_objective;
% y=JminObservBO'./true_objective;
% fig=figure();
% fig.Position=[200 0 2000 800];
% ax0=axes;
% ax0.FontSize=24;
% ax0.FontName='Times New Roman';
% [h,p,ci,stats] = ttest(x,y, 'Tail','left');
% addpath /home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/github_repo/boxplot2/
% skip=5;
% xx=3:skip:49;
% zz=cat(3,x(:,xx),y(:,xx));
% zz=permute(zz,[2,3,1]);
% h = boxplot2(zz,xx);
% cmap = [[1,0,0];[0,0,1]];
% for ii = 1:2
%     structfun(@(x) set(x(ii,:), 'color', cmap(ii,:), ...
%         'markeredgecolor', cmap(ii,:)), h);
% end
% set([h.lwhis h.uwhis], 'linestyle', '-');
% set([h.lwhis h.uwhis, h.box, h.med], 'linewidth', 2);
% % med=median(zz,3);
% med=quantile(y(:,xx),[0.25 0.75])';
% for itext=1:length(xx)
%     if itext==1
%         htext=text(xx(itext)-0.05, 1.01, sprintf('p-value=%.1e',p(xx(itext))), 'FontName', 'Times New Roman', 'FontSize', 22);
%     else
%         htext=text(xx(itext)-0.05, med(itext,2)*1.1, sprintf('p-value=%.1e',p(xx(itext))), 'FontName', 'Times New Roman', 'FontSize', 22);
% 
%     end
% set(htext,'Rotation',90);
% end
% grid on
% ylim([1 3])
% %xlim([0, 10])
% xticks(xx)
% set(h.out, 'marker', '.');
% xlabel(ax0, 'Iteration')
% ylabel(ax0, 'Optimality Ratio')
% % ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% set(gca,'yscale','log')
% figName=append(dir, idName,'_ORi_boxPval.png');
% saveas(gcf,figName)
% figName=append(dir, idName,'_ORi_boxPval.fig');
% saveas(gcf,figName)

fig=figure();
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on
% h1=semilogy(ax1, JminObservGBO(:,:)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', .5);
% h2=semilogy(ax1, JminObservBO(:,:)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', .5);
h3=semilogy(ax1, meanJminObservGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 4);
h4=semilogy(ax1, meanJminObservBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 4);

h5=yline(50.81,'k--', 'LineWidth', 3);

legend([h3, h4, h5],{'Guided BO: Minimum Observed Evaluation', 'BO: Minimum Observed Evaluation', 'Nominal Controller'}, 'Location', 'northeast');
grid on
% ylim(ax1, [1 3])
xlim([1, 50])

% for nominal at gains_nom= [0.4873, 1.5970]
grid minor
% x=1:50;
% y=TraceGBO.post_mus(2:end);
% err=TraceGBO.post_sigma2s(2:end);
% h9=errorbar(x, y, err, '-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red', 'LineStyle','none');


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
ylabel(ax1, 'Cost')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
set(ax1,'yscale','log')
figName=append(dir, idName,'_experiment.png');
saveas(gcf,figName)
figName=append(dir, idName,'_experiment.fig');
saveas(gcf,figName)

% %%
% fig=figure();
% fig.Position=[200 0 1600 800];
% ax1=axes;
% ax1.FontSize=24;
% ax1.FontName='Times New Roman';
% hold on
% % h1=semilogy(ax1, JminObservGBO(:,:)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', .5);
% % h2=semilogy(ax1, JminObservBO(:,:)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', .5);
% % h3=semilogy(ax1, meanJminObservGBO./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 5);
% x=(1:50);
% h1=bar(x,[meanJminObservGBO,meanJminObservBO]);
% % h2=bar(x2,meanJminObservBO, 'FaceColor', [0, 0, 1]);
% 
% 
% % h4=semilogy(ax1, meanJminObservBO./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 5);
% 
% legend([h1, h2],{'BO: Minimum Observedr Evaluation', 'BO: Minimum Observed Evaluation'}, 'Location', 'best');
% grid on
% % ylim(ax1, [1 3])
% %xlim([0, 10])
% 
% xlabel(ax1, 'Iteration')
% ylabel(ax1, 'Cost')
% % ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% set(ax1,'yscale','log')
% figName=append(dir, idName,'_bars.png');
% saveas(gcf,figName)
% figName=append(dir, idName,'_bars.fig');
% saveas(gcf,figName)

%% plot step response per various benchmark tunings
% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
G = tf(num, den, 'InputDelay',Td);

fig=figure();
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on

load(append(dir, 'exp_Data_37'))
y_GBO=exp_Data(518:830,4);
load(append(dirBO, 'exp_Data_10'))
y_BO=exp_Data(354:666,4);
load(append(dirBO, 'exp_Data_9'))
y_nom=exp_Data(402:714,4);
Tf=3.12;
time=0:0.01:Tf;

h1=plot(time, y_nom, 'k', 'LineWidth', 3);
h2=plot(time, y_BO, 'b', 'LineWidth', 3);
h3=plot(time, y_GBO, 'r', 'LineWidth', 3);
h4=yline(100,'k--', 'LineWidth', 3);

legend([h1, h2, h3, h4],{'Nominal PGM', 'BO', 'Guided BO', 'Reference'}, 'Location', 'southeast');
grid on
xlim(ax1, [0 1.8])
ylim(ax1, [80 102])

xlabel(ax1, 'Time')
ylabel(ax1, 'Velocity')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
set(gca,'yscale','log')
figName=append(dir, idName,'_experiment_data_response.png');
saveas(gcf,figName)
figName=append(dir, idName,'_experiment_data_response.fig');
saveas(gcf,figName)



end



function [objective, constraints] = ObjFun(X, G)

%     todo move some lines outside with handler@: faster?
C=tf([X(1), X(1)*X(2)], [1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.6]).RiseTime;
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
% w=[91.35, 0.34, 0.028, 0.0019];
% w=[40.	0.10	0.01	0.0002];

w=w./sum(w);
objective=ov/w(1)+st/w(2)+Tr/w(3)+ITAE/w(4);
constraints=-1;
end