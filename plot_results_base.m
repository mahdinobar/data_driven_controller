function plot_results_base
close all; clear; clc;
% hyper-params
idName= '0';
sys='robot_arm';
N0=1;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_0/';

tmp=[];
load(append(dir,'/InitobjectiveData_all.mat'))
objectiveData_all_0=InitobjectiveData_all(N0+1:end,1);
objectiveData_all_0=reshape(objectiveData_all_0(1:end),[N_iter,repeat_experiment]);

f2=figure(2);hold on
f2.Position=[200 0 1600 800];

for i=1:repeat_experiment
    for j=1:N_iter
        objectiveData_all_0(j,i)=nanmin(objectiveData_all_0(1:j,i));
    end
end

true_objective=0.1882; %with ts=1
% true_objective=0.1243; %with ts=0.1

for i=1:repeat_experiment
    semilogy(objectiveData_all_0(:,i)./true_objective, ':', 'LineWidth', 1, 'Color', [0, 0, 1, .5])
end

mean_objectiveData_all_0=mean(objectiveData_all_0,2,'omitnan');

hmean_0=semilogy(mean_objectiveData_all_0./true_objective, 'Color', [0, 0, 1, 1], 'LineWidth', 3, 'DisplayName','BO');

legend([hmean_0],{'BO'}, 'Location', 'best')
grid on
ylim([1 2])
xlabel('Iteration')
ylabel('Optimality Ratio')
title(append('Minimum Observed Objective vs Iterations over Real Plant (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times')
set(gca,'yscale','log')
figName=append(dir, idName,'.png');
saveas(gcf,figName)
figName=append(dir, idName,'.fig');
saveas(gcf,figName)

% robot-arm System ("Simultaneous computation of model order..., Badaruddin Muhammad et al.")
num = [-0.0118, 0.0257, 0, 0, 0];
den = [1, -3.1016, 4.3638, -3.1528, 1.0899, -0.0743];
ts=1;
G = d2c(tf(num,den, ts));
Tf=1e-8;
load(append(dir,'/InitData_all.mat'))
InitData_all=InitData_all{N0+1:end,:};
InitData_all=reshape(InitData_all(1:end,:),[N_iter,repeat_experiment,3]);
experiment=2;
last_gains=InitData_all(end, experiment, :);
C=tf([last_gains(3)+Tf*last_gains(1),last_gains(1)+Tf*last_gains(2),last_gains(2)], [Tf, 1, 0]);
CL=feedback(C*G, 1);
reference=1;

% % plot output error over time: e=|y-r| vs t
% plot_et(CL, reference, idName, dir)
% % plot y,r vs t
% plot_yrt(CL, reference, idName, dir)

for exper=1:repeat_experiment
    metrics=[];
    for i=1:N_iter
        gainsi=InitData_all(i, exper, :);
        Ci=tf([gainsi(3)+Tf*gainsi(1),gainsi(1)+Tf*gainsi(2),gainsi(2)], [Tf, 1, 0]);
        CLi=feedback(Ci*G, 1);
        metrics=[metrics; calc_metrics(CLi, reference)];
    end
    plot_metrics(metrics)
%     figName=append(dir, idName,'_metrics.png');
%     saveas(gcf,figName)
%     pause;
end
% metrics=reshape(metrics(1:end,:),[N_iter,repeat_experiment,6]);
% 
% 
% % plot metrics vs iteration
% plot_metrics(metrics, idName, dir)



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

function plot_metrics(metrics)
color=rand(1,3);
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
ylabel('Maximum Overshoot')
title('Maximum Overshoot')

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
title(append('Reference Tracking Error vs Time'))
figName=append(dir, idName,'_et.png');
saveas(gcf,figName)
pause;
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
title(append('System Response and Setpoint vs Time'))
figName=append(dir, idName,'_yrt.png');
saveas(gcf,figName)
pause;
end