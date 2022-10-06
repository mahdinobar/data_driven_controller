function plots_hyperparams
% plot surrogate rmse over number of data updates
close all;
clear all;
clc
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/G2rmse.mat')
fig1=figure(1);
fig1.Position=[200 0 1000 800];
ax1=axes;
y = mean(G2rmse,2)'; % your mean vector;
x = 1:numel(y);
std_dev = std(G2rmse');
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax1, x2, inBetween, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
hold on;
plot(ax1, x, y, 'k', 'LineWidth', 2);
xticks([1,2:15])
xlim([1,15])
grid on
ax1.FontSize=24;
ax1.FontName='Times New Roman';
set(gca,"FontSize",24)
set(gca,"FontName","Times")
xlabel(ax1, 'Number of Surrogate Update')
ylabel(ax1, 'RMSE')


fig2=figure(2);
fig2.Position=[200 0 1200 700];
ax2=axes;
ax2.FontSize=24;
ax2.FontName='Times New Roman';
hold on;
% according to experiment 71 and J_optimum~0.68 (instead of J_grid_optimum=0.545 due to noise existance)
S1=[2,5,7,9,11,15,20];
conv_iter_1=[19,15,14,10,13,16,20];
conv_iter_2=[20, 20, 22, 13, 15, 15, 20];
conv_iter_3=[20, 15, 25, 21, 29, 18, 16];
h1=plot(ax2, S1, conv_iter_1,'--o', 'LineWidth', 2);
h2=plot(ax2, S1, conv_iter_2,'--o', 'LineWidth', 2);
h3=plot(ax2, S1, conv_iter_3,'--o', 'LineWidth', 2);
h4=plot(ax2, S1, 22.*ones(size(S1)),'b--o', 'LineWidth', 3);
legend([h1,h2,h3, h4],{'GBO, S2=1','GBO, S2=2', 'GBO, S2=3', 'BO'}, 'Location', 'northeast'); 
grid on
xticks(S1)
xlim([2,20])
xlabel(ax2, 'S1')
ylabel(ax2, 'Number of Iterations to Converge')

end