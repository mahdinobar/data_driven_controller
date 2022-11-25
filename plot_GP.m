function plot_GP()
clc
close all;
clear all; 
% load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/trace_file_BO.mat')
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_4/trace_file_BO.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_4/grount_truth.mat")

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
plot(ax1, x, y, 'r', 'LineWidth', 2);
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax1, 'Iteration')
ylabel(ax1, 'Uncertainty(\sigma_{GP}^2)')
ylim([-1 1])

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
plot(ax2, x, y, 'r', 'LineWidth', 2);
ylim([-1 1])
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax2, 'Iteration')
ylabel(ax2, 'Uncertainty(\mu_{GP}-J)')

fig=figure(3);
fig.Position=[200 0 1000 800];
ax3=axes;
ax3.FontSize=24;
ax3.FontName='Times New Roman';
hold on
exper=1;
N0=1;
[x,idx] = sort(Trace(exper).hyper_grid_record(:,1)');
y=Trace(exper).post_mus_record(:,end)';
y=y(idx);
std_dev=Trace(exper).post_sigma2s_record(:,end)';
std_dev=std_dev(idx);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax3, x2, inBetween, 'y');
hold on; grid on;
plot(ax3, x, y, 'r', 'LineWidth', 2);
% errorbar(ax3, x,y,std_dev)
v=values(1+N0:end,exper);
p=samples_P(1+N0:end,exper);
scatter(p,v,'ok','filled')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax3, 'P gain')
ylabel(ax3, 'latent value f(P,I)')
title("GP model of BO after 50 Iterations")
% ylim([-1 1])

fig=figure(4);
fig.Position=[200 0 1000 800];
ax4=axes;
ax4.FontSize=24;
ax4.FontName='Times New Roman';
hold on
exper=1;
N0=1;
[x,idx] = sort(Trace(exper).hyper_grid_record(:,2)');
y=Trace(exper).post_mus_record(:,end)';
y=y(idx);
std_dev=Trace(exper).post_sigma2s_record(:,end)';
std_dev=std_dev(idx);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(ax4, x2, inBetween, 'y');
hold on; grid on;
plot(ax4, x, y, 'r', 'LineWidth', 2);
v=values(1+N0:end,exper);
p=samples_I(1+N0:end,exper);
scatter(p,v,'ok','filled')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax4, 'I gain')
ylabel(ax4, 'latent value f(P,I)')
title("GP model of BO after 50 Iterations")


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
view(3)
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

end
