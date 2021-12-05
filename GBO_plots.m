function GBO_plots(ms, mv, Trace, gain_mins, gain_maxes, idName, G)
% hyper-params
N0=3;
N_iter=30;
repeat_experiment=1;

dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/', idName, '/');

f1=figure();hold on
f1.Position=[200 0 1600 800];

% true_objective DC motor numeric
true_objective=3.1672;


h1=semilogy(Trace.values./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3, 'DisplayName','BO');
legend([h1],{'BO'}, 'Location', 'best')
grid on
% ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
% ylabel('Cost function')
title(append('Minimum Observed Objective vs Iterations over Real Plant (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'.png');
saveas(gcf,figName)
figName=append(dir, idName,'.fig');
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
plot_metrics(metrics, Trace, gain_mins, gain_maxes, dir, idName)
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

function plot_metrics(metrics, Trace, gain_mins, gain_maxes, dir, idName)
color=rand(1,3);
fig=figure();
fig.Position=[200 0 1600 800];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
subplot(3,2,1)
hold on;
plot(metrics(:,1),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(metrics(:,2),'o', 'MarkerFaceColor', color);
legend('BO')
grid on
xlabel('iteration')
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(metrics(:,3),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('T_r')
title('Rise Time from 10% to 100% of Reference')

subplot(3,2,4)
hold on;
plot(metrics(:,4),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(metrics(:,5),'o', 'MarkerFaceColor', color);
grid on
xlabel('iteration')
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(metrics(:,6),'o', 'MarkerFaceColor', color);
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
plot(Trace.samples(:, 1), metrics(:,1),'o', 'MarkerFaceColor', color);
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(Trace.samples(:, 1), metrics(:,2),'o', 'MarkerFaceColor', color);
legend('BO')
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(Trace.samples(:, 1), metrics(:,3),'o', 'MarkerFaceColor', color);
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('T_r')
title('Rise Time from 10% to 100% of Reference')

subplot(3,2,4)
hold on;
plot(Trace.samples(:, 1), metrics(:,4),'o', 'MarkerFaceColor', color);
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(Trace.samples(:, 1), metrics(:,5),'o', 'MarkerFaceColor', color);
grid on
xlabel('Kp')
xlim([gain_mins(1), gain_maxes(1)])
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(Trace.samples(:, 1), metrics(:,6),'o', 'MarkerFaceColor', color);
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
plot(Trace.samples(:, 2), metrics(:,1),'o', 'MarkerFaceColor', color);
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('e_m_a_x')
title('Maximum absolute tracking error')

subplot(3,2,2)
hold on;
plot(Trace.samples(:, 2), metrics(:,2),'o', 'MarkerFaceColor', color);
legend('BO')
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('T_s')
title('Settling Time')

subplot(3,2,3)
hold on;
plot(Trace.samples(:, 2), metrics(:,3),'o', 'MarkerFaceColor', color);
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('T_r')
title('Rise Time from 10% to 100% of Reference')

subplot(3,2,4)
hold on;
plot(Trace.samples(:, 2), metrics(:,4),'o', 'MarkerFaceColor', color);
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('ITAE')
title('Integral Time Absolute Error')

subplot(3,2,5)
hold on;
plot(Trace.samples(:, 2), metrics(:,5),'o', 'MarkerFaceColor', color);
grid on
xlabel('Ki')
xlim([gain_mins(2), gain_maxes(2)])
ylabel('ess')
title('Steady-state Error')

subplot(3,2,6)
hold on;
plot(Trace.samples(:, 2), metrics(:,6),'o', 'MarkerFaceColor', color);
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