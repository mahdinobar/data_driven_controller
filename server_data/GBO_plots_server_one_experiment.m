function GBO_plots_server_one_experiment
clear all;
clc;
close all;
N0=10;
N_iter=60;
N_G=5;
experiment=1; %plot which experiment?
ms=[0 0];
% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

idName='results_1/';
dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_1/';
dir_gains= append('./DC_motor_gain_bounds', '/', 'KpKi_bounds.mat');
load(dir_gains,'Kp_min','Kp_max', 'Ki_min', 'Ki_max')
mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
load(append(dir, idName,'trace_file.mat'), 'Trace')
TraceGBO=Trace(experiment);
clear Trace
% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 2.43936437324367;
ms_true=[0.6119, 1.6642];

JminObserv=TraceGBO.values(N0+1:end);
for j=N0+1:N_iter
    JminObserv(j-N0)=nanmin(TraceGBO.values(N0+1:j));
end
fig=figure();hold on
fig.Position=[200 0 1600 800];
h1=semilogy(TraceGBO.values(N0+1:end)./true_objective, '--', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
h2=semilogy(JminObserv./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 3);

% JminEst=TraceGBO.values(N0+1:end);
% for j=N0+1:N_iter
%     JminEst(j-N0)=nanmin(TraceGBO.post_mus(N0+1:j));
% end
% h5=semilogy(TraceGBO.post_mus(N0+1:end)./true_objective, 'Color', [0, 1, 0, 1], 'LineWidth', 1);
% h6=semilogy(JminEst./true_objective, 'Color', [0, 1, 0, 1], 'LineWidth', 3);

%--------------------------------------------------------------------------
% todo automatize code
load(append(dir, idName,'trace_file_BO.mat'), 'Trace')
TraceBO=Trace(experiment);
clear Trace
JminObserv=TraceBO.values(N0+1:end);
for j=N0+1:N_iter
    JminObserv(j-N0)=nanmin(TraceBO.values(N0+1:j));
end
h3=semilogy(TraceBO.values(N0+1:end)./true_objective, '--', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
h4=semilogy(JminObserv./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3);
legend([h1, h2, h3, h4],{'GBO: Minimum Evaluated','GBO: Minimum Observed Evaluation', 'BO: Minimum Evaluated','BO: Minimum Observed Evaluation'}, 'Location', 'best');
%--------------------------------------------------------------------------
% legend([h1, h2],{'GBO: Minimum Estimated','GBO: Minimum Observed'}, 'Location', 'best')
grid on
% ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_ORi.png');
saveas(gcf,figName)
figName=append(dir, idName,'_ORi.fig');
saveas(gcf,figName)


JminObserv=TraceGBO.values(N0+1:end);
for j=N0+1:N_iter
    JminObserv(j-N0)=nanmin(TraceGBO.values(N0+1:j));
end
fig=figure();hold on
fig.Position=[200 0 1600 800];
h1=plot(TraceGBO.values(N0+1:end), '--', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
h2=plot(JminObserv, 'Color', [1, 0, 0, 1], 'LineWidth', 3);
% JminEst=TraceGBO.values(N0+1:end);
% for j=N0+1:N_iter
%     JminEst(j-N0)=nanmin(TraceGBO.post_mus(N0+1:j));
% end
h5=plot(TraceGBO.post_mus(N0+1:end), 'Color', [0, 1, 0, 1], 'LineWidth', 3);
y=TraceGBO.post_mus(N0+1:end);
CI=TraceGBO.post_sigma2s(N0+1:end)/2;
x=linspace(1,length(y),length(y))';
h6=plot(y-CI, 'Color', [0, 1, 0, 1], 'LineWidth', 1);
plot(y+CI, 'Color', [0, 1, 0, 1], 'LineWidth', 1)

h7=plot(TraceBO.post_mus(N0+1:end), 'Color', [0, 0, 0, 1], 'LineWidth', 3);
y=TraceBO.post_mus(N0+1:end);
CI=TraceBO.post_sigma2s(N0+1:end)/2;
x=linspace(1,length(y),length(y))';
h8=plot(y-CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1);
plot(y+CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1)

x=0:N_G:N_iter-N0-1;
y=TraceGBO.G2_post_mus;
err=TraceGBO.G2_post_sigma2s;
h9=errorbar(x, y, err, '-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red', 'LineStyle','none');
% h6=semilogy(JminEst./true_objective, 'Color', [0, 1, 0, 1], 'LineWidth', 3);
%--------------------------------------------------------------------------
% todo automatize code
% load(append(dir,'trace_file_BO.mat'))
% TraceBO=Trace;
% clear Trace
JminObserv=TraceBO.values(N0+1:end);
for j=N0+1:N_iter
    JminObserv(j-N0)=nanmin(TraceBO.values(N0+1:j));
end
h3=plot(TraceBO.values(N0+1:end), '--', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
h4=plot(JminObserv, 'Color', [0, 0, 1, 1], 'LineWidth', 3);
legend([h1, h2, h3, h4, h5, h6, h7, h8, h9],{'GBO: Minimum Evaluated', ...
    'GBO: Minimum Observed Evaluation', 'BO: Minimum Evaluated', ...
    'BO: Minimum Observed Evaluation', 'GBO: Prediction', ...
    'GBO: 95% Confidence Interval', 'BO: Prediction', ...
    'BO: 95% Confidence Interval', 'GBO: Simulation Prediction with Confidence Bar'}, 'Location', 'southeast');
%--------------------------------------------------------------------------
grid on
% ylim([-100 100])
xlabel('Iteration')
ylabel('J')
% ylabel('Cost function')
title(append('Objective vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
figName=append(dir, idName,'_Ji.png');
saveas(gcf,figName)
figName=append(dir, idName,'_Ji.fig');
saveas(gcf,figName)

fig=figure();hold on
fig.Position=[200 0 1600 800];
c = linspace(1,N_iter-N0,N_iter-N0); 
h=scatter(TraceGBO.samples(N0+1:end, 1), TraceGBO.values(N0+1:end)./true_objective,[],c,'filled');
h2=scatter(TraceBO.samples(N0+1:end, 1), TraceBO.values(N0+1:end)./true_objective,[],c,'filled', '*');
cbar = colorbar;
colormap hot
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
ylabel('Optimality Ratio')
xlim([gain_mins(1), gain_maxes(1)])
% ylabel('Cost function')
title(append('Objective vs Kp gain (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_JKp.png');
saveas(gcf,figName)
figName=append(dir, idName,'_JKp.fig');
saveas(gcf,figName)

fig=figure();hold on
fig.Position=[200 0 1600 800];
h=scatter(TraceGBO.samples(N0+1:end, 2), TraceGBO.values(N0+1:end)./true_objective,[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
ylabel('Optimality Ratio')
xlim([gain_mins(2), gain_maxes(2)])
% ylabel('Cost function')
title(append('Objective vs Ki gain (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_JKi.png');
saveas(gcf,figName)
figName=append(dir, idName,'_JKi.fig');
saveas(gcf,figName)


% experiment=1; %pick an experiment to plot
C=tf([ms(1),ms(1)*ms(2)], [1, 0]);
CL=feedback(C*G, 1);

C_true=tf([ms_true(1),ms_true(1)*ms_true(2)], [1, 0]);
CL_true=feedback(C_true*G, 1);

% todo ?
reference=1; % reference signal

% plot output error over time: e=|y-r| vs t
plot_et(CL, reference, idName, dir)
% plot y,r vs t
plot_yrt(CL, CL_true, reference, idName, dir)

% plot metrics vs iteration
% for exper=1:repeat_experiment
metrics=[];
for i=1:N_iter
    gainsi=TraceGBO.samples(i, :);
    Ci=tf([gainsi(1),gainsi(1)*gainsi(2)], [1, 0]);
    CLi=feedback(Ci*G, 1);
    metrics=[metrics; calc_metrics(CLi, reference)];
end
plot_metrics(metrics, TraceGBO, gain_mins, gain_maxes, N0, dir, idName)
% end
pause;
close all;
end

function [metrics] = calc_metrics(CL, r)
[y,t]=step(CL);
e=abs(y-r);
emax=max(e);
ov=stepinfo(CL).Overshoot;
Ts=stepinfo(CL).SettlingTime;
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,1.0]).RiseTime;
ess= e(end);
% epsilon = sin(t*3 - pi/6).*exp(-0.2*t); %Example for epsilon
ITAE = trapz(t, t.*abs(e));
metrics=[emax, Ts, Tr, ITAE, ess, ov];
end

function plot_metrics(metrics, Trace, gain_mins, gain_maxes, N0, dir, idName)
color=rand(1,3);
fig=figure();
fig.Position=[200 0 1600 800];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
subplot(3,2,1)
hold on;
plot(metrics(1:N0,1),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,1),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(metrics(1:N0,2),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,2),'o', 'MarkerFaceColor', color);
legend('BO')
grid on
xlabel('iteration')
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(metrics(1:N0,3),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,3),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('T_r')
title('Rise Time from 10% to 98% of Reference')

subplot(3,2,4)
hold on;
plot(metrics(1:N0,4),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,4),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(metrics(1:N0,5),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,5),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(metrics(1:N0,6),'x', 'MarkerFaceColor', color);
plot(metrics(N0+1:end,6),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('max(0, 100*(y_{max}-r)')
title('Maximum Overshoot')

figName=append(dir, idName,'_metrics_iter.png');
saveas(gcf,figName)

% =========================================================================
fig=figure();
fig.Position=[200 0 1600 800];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
subplot(3,2,1)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,1),'x', 'MarkerFaceColor', color);
c = linspace(1,length(metrics(N0+1:end,1)),length(metrics(N0+1:end,1))); 
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,1),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,2),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,2),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,3),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,3),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('T_r')
title('Rise Time from 10% to 98% of Reference')

subplot(3,2,4)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,4),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,4),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,5),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,5),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(Trace.samples(1:N0, 1), metrics(1:N0,6),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 1), metrics(N0+1:end,6),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('max(0, 100*(y_{max}-r)')
title('Maximum Overshoot')

figName=append(dir, idName,'_metrics_Kp.png');
saveas(gcf,figName)

% =========================================================================
fig=figure();
fig.Position=[200 0 1600 800];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
subplot(3,2,1)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,1),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,1),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,2),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,2),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,3),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,3),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('T_r')
title('Rise Time from 10% to 98% of Reference')

subplot(3,2,4)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,4),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,4),[],c,'filled');
cbar = colorbar;
colormap copper
grid on
ylabel(cbar, 'iteration')
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,5),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,5),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(Trace.samples(1:N0, 2), metrics(1:N0,6),'x', 'MarkerFaceColor', color);
scatter(Trace.samples(N0+1:end, 2), metrics(N0+1:end,6),[],c,'filled');
cbar = colorbar;
colormap copper
ylabel(cbar, 'iteration')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('max(0, 100*(y_{max}-r)')
title('Maximum Overshoot')

figName=append(dir, idName,'_metrics_Ki.png');
saveas(gcf,figName)

end

function plot_et(TF, r, idName, dir)
% plot e over t
fig=figure();
hold on;
fig.Position=[200 0 1600 800];
[y,t]=step(TF);
graph=plot(t, abs(y-r),'Color', [0, 0, 1, 1], 'LineWidth', 2);
legend([graph],{'Tracking Error'}, 'Location', 'best')
grid on
xlabel('Time (sec)')
ylabel('e=|y-r|')
title(append('Reference Tracking Error vs Time using Optimum Controller Gains'))
figName=append(dir, idName,'_et.png');
saveas(gcf,figName)
end

function plot_yrt(TF, TFtrue, r, idName, dir)
% plot e over t
fig=figure();
hold on;
fig.Position=[200 0 1600 800];
[y,t]=step(TF);
graph1=plot(t, y,'Color', [0, 0, 1, 1], 'LineWidth', 2);
graph2=plot(t, r.*ones(size(t)), ':', 'Color', [0.2, 0.2, 0.2, 1], 'LineWidth', 2);

[y_ture,t_ture]=step(TFtrue);
graph3=plot(t_ture, y_ture,'Color', [0, 1, 0, 1], 'LineWidth', 2);

legend([graph1, graph2, graph3],{'y: output', 'r: reference', 'ground true'}, 'Location', 'best')

grid on
xlabel('Time (sec)')
ylabel('y, r')
ymargin=0.05;
ylim([min([r;y])-ymargin,max([r;y])+ymargin]);
title(append('System Response and Setpoint vs Time using Optimum Controller Gains'))
figName=append(dir, idName,'_yrt.png');
saveas(gcf,figName)
end