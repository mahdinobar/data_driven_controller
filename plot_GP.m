function plot_GP()
clc
close all;
clear all; 
% load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/trace_file_BO.mat')
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/trace_file_BO.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/grount_truth.mat")

post_sigma2s=[];
post_mus=[];
values=[];
samples_P=[];
samples_I=[];
for i=1:2
    post_sigma2s=[post_sigma2s, Trace(i).post_sigma2s];
    post_mus=[post_mus, Trace(i).post_mus];
    values=[values, Trace(i).values];
    samples_P=[samples_P, Trace(i).samples(:,1)];
    samples_I=[samples_I, Trace(i).samples(:,2)];
end

fig=figure(1);
fig.Position=[200 0 1000 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on
y = mean(post_sigma2s,2); % your mean vector;
y=y(2:51)';
x = 1:50;
std_dev = var(post_sigma2s');
std_dev=std_dev(2:51);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax1, x2, inBetween, 'g');
hold on; grid on;
plot(ax1, x, y, '-ko', 'LineWidth', 2);
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax1, 'Iteration')
ylabel(ax1, '\sigma_{GP}^2')
ylim([0 5])
xlim([1 50])
title('GP uncertainty at sampled point')

fig2=figure(2);
fig2.Position=[200 0 1000 800];
ax2=axes;
ax2.FontSize=24;
ax2.FontName='Times New Roman';
hold on
diff=post_mus-values;
y = mean(diff,2); % your mean vector;
y=y(2:51)';
x = 1:50;
std_dev = var(diff');
std_dev=std_dev(2:51);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax2, x2, inBetween, 'g');
hold on; grid on;
plot(ax2, x, y, '-ko', 'LineWidth', 2);
ylim([-1 1])
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax2, 'Iteration')
ylabel(ax2, '\mu_{GP}-J')
title('GP uncertainty at sampled point')
ylim([-5 5])
xlim([1 50])

% fig=figure(3);
% fig.Position=[200 0 1000 800];
% ax3=axes;
% ax3.FontSize=24;
% ax3.FontName='Times New Roman';
% hold on
% exper=1;
% N0=1;
% [x,idx] = sort(Trace(exper).hyper_grid_record(:,1)');
% y=Trace(exper).post_mus_record(:,end)';
% y=y(idx);
% std_dev=Trace(exper).post_sigma2s_record(:,end)';
% std_dev=std_dev(idx);
% curve1 = y + std_dev;
% curve2 = y - std_dev;
% x2 = [x, fliplr(x)];
% inBetween = [curve1, fliplr(curve2)];
% fill(ax3, x2, inBetween, 'y');
% hold on; grid on;
% plot(ax3, x, y, 'r', 'LineWidth', 2);
% % errorbar(ax3, x,y,std_dev)
% v=values(1+N0:end,exper);
% p=samples_P(1+N0:end,exper);
% scatter(p,v,'ok','filled')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax3, 'P gain')
% ylabel(ax3, 'latent value f(P,I)')
% title("GP model of BO after 50 Iterations")
% % ylim([-1 1])
% 
% fig=figure(4);
% fig.Position=[200 0 1000 800];
% ax4=axes;
% ax4.FontSize=24;
% ax4.FontName='Times New Roman';
% hold on
% exper=1;
% N0=1;
% [x,idx] = sort(Trace(exper).hyper_grid_record(:,2)');
% y=Trace(exper).post_mus_record(:,end)';
% y=y(idx);
% std_dev=Trace(exper).post_sigma2s_record(:,end)';
% std_dev=std_dev(idx);
% curve1 = y + std_dev;
% curve2 = y - std_dev;
% x2 = [x, fliplr(x)];
% inBetween = [curve1, fliplr(curve2)];
% fill(ax4, x2, inBetween, 'y');
% hold on; grid on;
% plot(ax4, x, y, 'r', 'LineWidth', 2);
% v=values(1+N0:end,exper);
% p=samples_I(1+N0:end,exper);
% scatter(p,v,'ok','filled')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax4, 'I gain')
% ylabel(ax4, 'latent value f(P,I)')
% title("GP model of BO after 50 Iterations")


fig=figure(5);
fig.Position=[200 0 1000 800];
ax5=axes;
ax5.FontSize=24;
ax5.FontName='Times New Roman';
hold on;
grid on;
exper=2;
N0=1;
x=Trace(exper).hyper_grid_record(:,1);
y=Trace(exper).hyper_grid_record(:,2);
z=Trace(exper).post_mus_record(:,end);
xs=samples_P(1+N0:end,exper);
ys=samples_I(1+N0:end,exper);
zs=values(1+N0:end,exper);
% h1=surf(ax5, x,y,z);
dt = delaunayTriangulation(x,y) ;
tri = dt.ConnectivityList ;
xi = dt.Points(:,1) ; 
yi = dt.Points(:,2) ; 
F = scatteredInterpolant(x,y,z);
zi = F(xi,yi) ;
h1=trisurf(tri,xi,yi,zi,'FaceColor','black','FaceAlpha',0.3);
view(45,30)
% view(45,0)
% shading interp
h2=scatter3(xs,ys,zs,"or","filled", 'SizeData', 50);
h3=surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
set(gca,'zscale','log')
set(gca,'ColorScale','log')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax5, 'P gain')
ylabel(ax5, 'I gain')
zlabel(ax5, 'latent value f(P,I)')
title("GP model of BO after 50 Iterations")
legend([h1, h2, h3],{'GP model', 'BO samples', 'ground truth'}, 'Location', 'northeast'); 

fig=figure(6);
fig.Position=[200 0 1000 800];
ax6=axes;
ax6.FontSize=24;
ax6.FontName='Times New Roman';
hold on;
grid on;
exper=1;
hyp_GP_cov=Trace(exper).hyp_GP_cov;
hyp_GP_lik=Trace(exper).hyp_GP_lik;
hyp_GP_mean=Trace(exper).hyp_GP_mean;
h1=plot(ax6,hyp_GP_mean,'-o');
h2=plot(ax6,hyp_GP_cov,'-o');
h3=plot(ax6,hyp_GP_lik,'-o');
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
ylabel(ax6, 'GP hyperparameter')
xlabel(ax6, 'iteration')
xlim([1 50])
title("GP model Hyperparameters evolution")
legend([h1, h2(1), h2(2),h2(3), h3],{'@meanConst: c', '@covMaternard: \lambda_1', '@covMaternard: \lambda_2', '@covMaternard: \sigma_f', '@likGauss: ln(\sigma)'}, 'Location', 'best'); 

fig=figure(7);
fig.Position=[200 0 1000 800];
ax7=axes;
ax7.FontSize=24;
ax7.FontName='Times New Roman';
hold on;
grid on;
exper=1;
alpha=Trace(exper).GP_posterior_record.alpha;
sW=Trace(exper).GP_posterior_record.sW;
L=Trace(exper).GP_posterior_record.L;
h1=plot(ax7,alpha,'-o');
h2=plot(ax7,sW,'-o');
% h3=plot(ax7,L,'-o');
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
ylabel(ax7, 'posterior')
xlabel(ax7, 'iteration')
xlim([1 50])
title("GP approximate posterior")
legend([h1, h2],{'alpha', 'sW'}, 'Location', 'best'); 


fig=figure(8);
fig.Position=[200 0 1000 800];
ax8=axes;
ax8.FontSize=24;
ax8.FontName='Times New Roman';
hold on
exper=2;
y = mean(Trace(exper).post_sigma2s_record,1); % your mean vector;
x = 1:50;
std_dev = var(Trace(1).post_sigma2s_record,1);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax8, x2, inBetween, [.7 .7 .7]);
hold on; grid on;
plot(ax8, x, y, '-ko', 'LineWidth', 2);
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax8, 'Iteration')
ylabel(ax8, '\sigma_{GP}^2')
ylim([0 10])
xlim([1 50])
xticks([1, 5:5:50])
title('GP uncertainty at entire hyper grid recorded')


fig=figure(9);
fig.Position=[200 0 1000 800];
ax9=axes;
ax9.FontSize=24;
ax9.FontName='Times New Roman';
hold on
exper=2;
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/J_truth_hyper_grid_record_1.mat")
y = mean(Trace(exper).post_mus_record-j_pt); % your mean vector;
x = 1:50;
std_dev = var(Trace(exper).post_mus_record-j_pt);;
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax9, x2, inBetween, [.7 .7 .7]);
hold on; grid on;
plot(ax9, x, y, '-ko', 'LineWidth', 2);
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax9, 'Iteration')
ylabel(ax9, '\mu_{GP}-J')
ylim([-10 10])
xlim([1 50])
xticks([1, 5:5:50])
title('GP uncertainty at entire hyper grid recorded')

fig=figure(10);
fig.Position=[200 0 1000 800];
ax10=axes;
ax10.FontSize=24;
ax10.FontName='Times New Roman';
hold on
grid on
ratio=[];
exper=2;
for i=1:50
    aq_m=Trace(exper).AQ_vals(i);
    aq_M=max(Trace(exper).AQ_vals(1:i));
    ratio(end+1)=aq_m/aq_M;
end
plot(ax10, ratio, '-ko', 'LineWidth', 2);
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax10, 'Iteration')
ylabel(ax10, 'EI_{m} / max EI_{m}')
% ylim([-10 10])
xlim([1 50])
xticks([1, 5:5:50])
title('EI over maximum EI vs iteration')

end
