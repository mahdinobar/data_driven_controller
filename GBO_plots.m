function GBO_plots(ms, mv, Trace, gain_mins, gain_maxes, N0, N_iter, idName, G)

dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');
% true_objective DC motor numeric
% true_objective=3.1672;
true_objective = 2.43936437324367;

JminObserv=Trace.values(N0+1:end);
for j=N0+1:N_iter
    JminObserv(j-N0)=nanmin(Trace.values(N0+1:j));
end
fig=figure();hold on
fig.Position=[200 0 1600 800];
h1=semilogy(Trace.values(N0+1:end)./true_objective, '--', 'Color', [1, 0, 0, 1], 'LineWidth', 1);
h2=semilogy(JminObserv./true_objective, 'Color', [1, 0, 0, 1], 'LineWidth', 3);

% %--------------------------------------------------------------------------
% % todo automatize code
% load(append(dir,'trace_file_BO.mat'))
% JminObserv=Trace.values(N0+1:end);
% for j=N0+1:N_iter
%     JminObserv(j-N0)=nanmin(Trace.values(N0+1:j));
% end
% h3=semilogy(Trace.values(N0+1:end)./true_objective, '--', 'Color', [0, 0, 1, 1], 'LineWidth', 1);
% h4=semilogy(JminObserv./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3);
% legend([h1, h2, h3, h4],{'GBO: Minimum Estimated','GBO: Minimum Observed', 'BO: Minimum Estimated','BO: Minimum Observed'}, 'Location', 'best');
% %--------------------------------------------------------------------------

legend([h1, h2],{'GBO: Minimum Estimated','GBO: Minimum Observed'}, 'Location', 'best')
grid on
% ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
% ylabel('Cost function')
title(append('Objective vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'_Ji.png');
saveas(gcf,figName)
figName=append(dir, idName,'_Ji.fig');
saveas(gcf,figName)

fig=figure();hold on
fig.Position=[200 0 1600 800];
c = linspace(1,N_iter-N0,N_iter-N0); 
h=scatter(Trace.samples(N0+1:end, 1), Trace.values(N0+1:end)./true_objective,[],c,'filled');
cbar = colorbar;
colormap jet
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
h=scatter(Trace.samples(N0+1:end, 2), Trace.values(N0+1:end)./true_objective,[],c,'filled');
cbar = colorbar;
colormap jet
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

% todo ?
reference=1; % reference signal

% plot output error over time: e=|y-r| vs t
plot_et(CL, reference, idName, dir)
% plot y,r vs t
plot_yrt(CL, reference, idName, dir)

% plot metrics vs iteration
% for exper=1:repeat_experiment
metrics=[];
for i=1:N_iter
    gainsi=Trace.samples(i, :);
    Ci=tf([gainsi(1),gainsi(1)*gainsi(2)], [1, 0]);
    CLi=feedback(Ci*G, 1);
    metrics=[metrics; calc_metrics(CLi, reference)];
end
plot_metrics(metrics, Trace, gain_mins, gain_maxes, N0, dir, idName)
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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
colormap jet
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

function plot_yrt(TF, r, idName, dir)
% plot e over t
fig=figure();
hold on;
fig.Position=[200 0 1600 800];
[y,t]=step(TF);
graph1=plot(t, y,'Color', [0, 0, 1, 1], 'LineWidth', 2);
graph2=plot(t, r.*ones(size(t)), ':', 'Color', [0.2, 0.2, 0.2, 1], 'LineWidth', 2);

legend([graph1, graph2],{'y: output', 'r: reference'}, 'Location', 'best')

grid on
xlabel('Time (sec)')
ylabel('y, r')
ymargin=0.05;
ylim([min([r;y])-ymargin,max([r;y])+ymargin]);
title(append('System Response and Setpoint vs Time using Optimum Controller Gains'))
figName=append(dir, idName,'_yrt.png');
saveas(gcf,figName)
end