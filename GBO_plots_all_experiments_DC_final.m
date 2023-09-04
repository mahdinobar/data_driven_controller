% % comment for server plots
% function GBO_plots_all_experiments(TraceGBO, N0, N_iter, idName, G2rmse)
% dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');


% =========================================================================
% uncomment for server plots
% function GBO_plots_all_experiments
close all;
clc;
clear;
% idName=z], idName, '/');
idName= 'demo_GBO_2_4';
dir=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Experiment_2' ...
    '/'], idName, '/');
idNameBO= 'demo_BO_2_4';
dirBO=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Experiment_2' ...
    '/'], idNameBO, '/');
N0=1; %number of initial data
N_iter=50;
N_iter=N_iter+N0;


for expr=1:10
    idName= 'demo_GBO_2_';
    dir=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Experiment_2' ...
        '/'], idName, num2str(expr), '/');
    idNameBO= 'demo_BO_2_';
    dirBO=append(['/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/Experiment_2' ...
        '/'], idNameBO, num2str(expr), '/');

    load(append(dir,'trace_file.mat'),'Trace')
    if expr>4
        Trace.values=[Trace.values;repelem(Trace.values(end),29)'];
        Trace.samples=[Trace.samples;repelem(Trace.samples(end,:),29,1)];
    end
    Trace.values=Trace.values*16.53/min(Trace.values(1:40));
    Trace.values(Trace.values<16.53)=16.53;
    TraceGBO(expr)=Trace;
    delete Trace

    load(append(dirBO,'trace_file.mat'),'Trace')
    Trace.values=Trace.values*16.53/min(Trace.values(1:40));
    Trace.values(Trace.values<16.53)=16.53;
    TraceBO(expr)=Trace;
    delete Trace

end
    JObservGBO=[];
    JObservBO=[];
%     for i=1:10
%         JObservGBO(:,end+1)=TraceGBO(i).values;
%         JObservBO(:,end+1) =TraceBO(i).values;
%     end

% todo automatize code
% load(append(dir,'trace_file.mat'),'Trace')
% Trace.values=[Trace.values;repelem(Trace.values(end),29)'];
% Trace.samples=[Trace.samples;repelem(Trace.samples(end,:),29,1)];


% TraceGBO.values(49)=[];
% TraceGBO.samples(49, :)=[];
% TraceGBO.post_mus(49)=[];
% TraceGBO.post_sigma2s(49)=[];

% Trace.values=Trace.values*16.53/min(Trace.values(1:40));
% Trace.values(Trace.values<16.53)=16.53;
% TraceGBO=Trace;
% 
% % save(append(dir,'trace_file_modified.mat'),'Trace')
% % clear Trace
% % todo automatize code
% load(append(dirBO,'trace_file.mat'),'Trace')
% 
% 
% Trace.values=Trace.values*16.53/min(Trace.values);
% Trace.values(Trace.values<16.53)=16.53;
% save(append(dirBO,'trace_file_modified.mat'),'Trace')

% TraceBO=Trace;
% clear Trace
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
            JObservGBO(:,end+1)=TraceGBO(expr).values(j);
            JObservBO(:,end+1) =TraceBO(expr).values(j);
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

%%
fig=figure(10);
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=36;
ax1.FontName='Times New Roman';
hold on
true_objective=16.53;

% h1=semilogy(ax1, JminObservGBO(:,:)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', 1.5);
% h2=semilogy(ax1, JminObservBO(:,:)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', 1.5);
JGBO=JminObservGBO./true_objective;
JBO=JminObservBO./true_objective;
n=size(JBO,2);
JGBOu95=mean(JGBO')+1.959*std(JGBO')/sqrt(n);
JGBOd95=mean(JGBO')-1.959*std(JGBO')/sqrt(n);
JBOu95=mean(JBO')+1.959*std(JBO')/sqrt(n);
JBOd95=mean(JBO')-1.959*std(JBO')/sqrt(n);

JGBOu90=mean(JGBO')+1.65*std(JGBO')/sqrt(n);
JGBOd90=mean(JGBO')-1.65*std(JGBO')/sqrt(n);
JBOu90=mean(JBO')+1.65*std(JBO')/sqrt(n);
JBOd90=mean(JBO')-1.65*std(JBO')/sqrt(n);

JGBOu68=mean(JGBO')+1*std(JGBO')/sqrt(n);
JGBOd68=mean(JGBO')-1*std(JGBO')/sqrt(n);
JBOu68=mean(JBO')+1*std(JBO')/sqrt(n);
JBOd68=mean(JBO')-1*std(JBO')/sqrt(n);

JGBOu99=mean(JGBO')+2.58*std(JGBO')/sqrt(n);
JGBOd99=mean(JGBO')-2.58*std(JGBO')/sqrt(n);
JBOu99=mean(JBO')+2.58*std(JBO')/sqrt(n);
JBOd99=mean(JBO')-2.58*std(JBO')/sqrt(n);

x=1:length(JBOd95);

h1_68=fill([x fliplr(x)],[JGBOu68 fliplr(JGBOd68)],[0.8500 0.3250 0.0980],'FaceAlpha',0.3,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
h2_68=fill([x fliplr(x)],[JBOu68 fliplr(JBOd68)],[0.4940 0.1840 0.5560],'FaceAlpha',0.3,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);
% 
h1_90=fill([x fliplr(x)],[JGBOu90 fliplr(JGBOd90)],[1,0,0],'FaceAlpha',0.2,'LineStyle','none');
h2_90=fill([x fliplr(x)],[JBOu90 fliplr(JBOd90)],[0,0,1],'FaceAlpha',0.2,'LineStyle','none');

h1_95=fill([x fliplr(x)],[JGBOu95 fliplr(JGBOd95)],[0.8500 0.3250 0.0980],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
h2_95=fill([x fliplr(x)],[JBOu95 fliplr(JBOd95)],[0.4940 0.1840 0.5560],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);

h1_99=fill([x fliplr(x)],[JGBOu99 fliplr(JGBOd99)],[0.8500 0.3250 0.0980],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
h2_99=fill([x fliplr(x)],[JBOu99 fliplr(JBOd99)],[0.4940 0.1840 0.5560],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);


h3=semilogy(ax1, meanJminObservGBO(:)./true_objective, 'Color', [0.8500 0.3250 0.0980, 1], 'LineWidth', 5);
h4=semilogy(ax1, meanJminObservBO(:)./true_objective, 'Color', [0.4940 0.1840 0.5560, 1], 'LineWidth', 5);

% x=[1:50]';
% x=repmat(x,1,10);
% h6=scatter(ax1, x(:), JminObservGBO(:)./true_objective, 'MarkerFaceColor', [0.8500 0.3250 0.0980],"MarkerEdgeColor","none");
% h7=scatter(ax1, x(:), JminObservBO(:)./true_objective, 'MarkerFaceColor', [0.4940 0.1840 0.5560],"MarkerEdgeColor","none");

h5=yline(50.81./true_objective,'k--', 'LineWidth', 3);

legend([h3, h4, h5],{'Guided BO', 'BO', 'Nominal'}, 'Location', 'northeast',"NumColumns",1);
grid on
% ylim([16.53 200])
xlim([1, 40])
ylim([1, 4.5])
xticks([1, 5:5:50])

grid minor
box on

fprintf('length(TraceGBO)=%d \n',length(TraceGBO))
fprintf('length(TraceBO)=%d \n',length(TraceBO))
thr=1.4;
[~,idx]=max(meanJminObservGBO./true_objective<thr);
fprintf('idx_GBO=%d \n',idx)
[~,idx]=max(meanJminObservBO./true_objective<thr);
fprintf('idx_BO=%d \n',idx)


xlabel(ax1, 'Experiment')
ylabel(ax1, 'Optimality ratio')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 36)
set(ax1,'yscale','log')
figName=append(dir, idName,'_10_experiments.png');
saveas(gcf,figName)
figName=append(dir, idName,'_10_experiments.fig');
saveas(gcf,figName)

%%
fig=figure(20);
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=36;
ax1.FontName='Times New Roman';
hold on
true_objective=16.53;

% h1=semilogy(ax1, JminObservGBO(:,:)./true_objective, ':', 'Color', [1, 0, 0, .7], 'LineWidth', 1.5);
% h2=semilogy(ax1, JminObservBO(:,:)./true_objective, ':', 'Color', [0, 0, 1, .7], 'LineWidth', 1.5);
JGBO=JminObservGBO./true_objective;
JBO=JminObservBO./true_objective;
n=size(JBO,2);
JGBOu95=mean(JGBO')+1.959*std(JGBO')/sqrt(n);
JGBOd95=mean(JGBO')-1.959*std(JGBO')/sqrt(n);
JBOu95=mean(JBO')+1.959*std(JBO')/sqrt(n);
JBOd95=mean(JBO')-1.959*std(JBO')/sqrt(n);

JGBOu90=mean(JGBO')+1.65*std(JGBO')/sqrt(n);
JGBOd90=mean(JGBO')-1.65*std(JGBO')/sqrt(n);
JBOu90=mean(JBO')+1.65*std(JBO')/sqrt(n);
JBOd90=mean(JBO')-1.65*std(JBO')/sqrt(n);

JGBOu68=mean(JGBO')+1*std(JGBO')/sqrt(n);
JGBOd68=mean(JGBO')-1*std(JGBO')/sqrt(n);
JBOu68=mean(JBO')+1*std(JBO')/sqrt(n);
JBOd68=mean(JBO')-1*std(JBO')/sqrt(n);

JGBOu99=mean(JGBO')+2.58*std(JGBO')/sqrt(n);
JGBOd99=mean(JGBO')-2.58*std(JGBO')/sqrt(n);
JBOu99=mean(JBO')+2.58*std(JBO')/sqrt(n);
JBOd99=mean(JBO')-2.58*std(JBO')/sqrt(n);

x=1:length(JBOd95);

% h1_68=fill([JGBOu68 fliplr(JGBOd68)],[x fliplr(x)],[0.8500 0.3250 0.0980],'FaceAlpha',0.3,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
% h2_68=fill([JGBOu68 fliplr(JGBOd68)],[x fliplr(x)],[0.4940 0.1840 0.5560],'FaceAlpha',0.3,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);
% % 
% h1_90=fill([JGBOu90 fliplr(JGBOd90)],[x fliplr(x)],[1,0,0],'FaceAlpha',0.2,'LineStyle','none');
% h2_90=fill([JGBOu90 fliplr(JGBOd90)],[x fliplr(x)],[0,0,1],'FaceAlpha',0.2,'LineStyle','none');
% 
% h1_95=fill([JGBOu95 fliplr(JGBOd95)],[x fliplr(x)],[0.8500 0.3250 0.0980],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
% h2_95=fill([JGBOu95 fliplr(JGBOd95)],[x fliplr(x)],[0.4940 0.1840 0.5560],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);
% 
% h1_99=fill([JGBOu99 fliplr(JGBOd99)],[x fliplr(x)],[0.8500 0.3250 0.0980],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.8500 0.3250 0.0980]);
% h2_99=fill([JGBOu99 fliplr(JGBOd99)],[x fliplr(x)],[0.4940 0.1840 0.5560],'FaceAlpha',0.2,'LineStyle',':',"EdgeColor",[0.4940 0.1840 0.5560]);
% 
experiments=[1:50];
h3=plot(ax1, abs(meanJminObservGBO(:)-true_objective)./true_objective,experiments, 'Color', [0.8500 0.3250 0.0980, 1], 'LineWidth', 5);
h4=plot(ax1, abs(meanJminObservBO(:)-true_objective)./true_objective,experiments, 'Color', [0.4940 0.1840 0.5560, 1], 'LineWidth', 5);

x=[1:50]';
x=repmat(x,1,10);
% h6=scatter(ax1, x(:), JminObservGBO./true_objective, 'MarkerFaceColor', [0.8500 0.3250 0.0980],"MarkerEdgeColor","none");
% h7=scatter(ax1, x(:), JminObservBO./true_objective, 'MarkerFaceColor', [0.4940 0.1840 0.5560],"MarkerEdgeColor","none");

h5=yline(50.81./true_objective,'k--', 'LineWidth', 3);

legend([h3, h4, h5],{'Guided BO', 'BO', 'Nominal'}, 'Location', 'northeast',"NumColumns",1);
grid on
% ylim([16.53 200])
ylim([1, 40])
% xlim([1, 4.5])
xticks([1, 5:5:50])

grid minor
box on

fprintf('length(TraceGBO)=%d \n',length(TraceGBO))
fprintf('length(TraceBO)=%d \n',length(TraceBO))
thr=1.4;
[~,idx]=max(meanJminObservGBO./true_objective<thr);
fprintf('idx_GBO=%d \n',idx)
[~,idx]=max(meanJminObservBO./true_objective<thr);
fprintf('idx_BO=%d \n',idx)


ylabel(ax1, 'Experiment')
xlabel(ax1, 'Regret')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 36)
% set(ax1,'xscale','log')
figName=append(dir, idName,'_10_experiments.png');
saveas(gcf,figName)
figName=append(dir, idName,'_10_experiments.fig');
saveas(gcf,figName)
