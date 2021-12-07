function draft()
close all; clear;
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_1_1/trace_file_BO.mat')
TraceBO=Trace;
clear Trace
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_1_1/trace_file.mat')
TraceGBO=Trace;
clear Trace

N_iter=33;
N0=3;
true_objective = 2.43936437324367;

fig=figure();hold on
fig.Position=[200 0 1600 800];
grid on
% c = linspace(1,N_iter-N0,N_iter-N0);
% h=scatter(TraceGBO.samples(N0+1:end, 1), TraceGBO.values(N0+1:end)./true_objective,[],c,'filled', 'SizeData', 100);
% h2=scatter(TraceBO.samples(N0+1:end, 1), TraceBO.values(N0+1:end)./true_objective,[],c, 'filled', '^', 'SizeData', 100);

h3=plot(TraceGBO.post_mus(N0+1:end), 'Color', [0, 1, 0, 1], 'LineWidth', 3);
y=TraceGBO.post_mus(N0+1:end);
CI=TraceGBO.post_sigma2s(N0+1:end)/2;
x=linspace(1,length(y),length(y))';

plot(y-CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1)
plot(y+CI, 'Color', [0, 0, 0, 1], 'LineWidth', 1)

x=0:5:29;
y=TraceGBO.G2_post_mus;
err=TraceGBO.G2_post_sigma2s;
errorbar(x, y, err, '-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','red', 'LineStyle','none');


% fill([x;fliplr(x)], [y-CI;fliplr(y+CI)], [0, 1, 0], 'FaceAlpha',0.5)

% cbar = colorbar;
% colormap winter
% ylabel(cbar, 'iteration')
% legend([h, h2],{'GBO', 'BO'}, 'Location', 'best');
% grid on
% xlabel('Kp')
% ylabel('Optimality Ratio')
% xlim([gain_mins(1), gain_maxes(1)])
% % ylabel('Cost function')
% title(append('Objective vs Kp gain (N0=',num2str(N0),')'))

end
