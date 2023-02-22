close all;
clear all;
clc;
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/gt/grid_pt.mat')
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_72_eta2_02_eta1_3/results_1/trace_file.mat')
fig=figure(1);

surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
xlabel('Kp')
ylabel('Ki')
zlabel('J')
set(gca,'zscale','log')
set(gca,'ColorScale','log')
colormap(gca,"winter")
colorbar
hold on
view(2)
exper=2;
N0=1;
kp = Trace(exper).samples(1+N0:end,1);
ki = Trace(exper).samples(1+N0:end,2);
c = linspace(1,10,length(kp));
z=max(j_pt,[],'all').*ones(length(kp),1);
scatter3(kp,ki,z,[],c,'filled')
colormap(gca,"parula")
colorbar












fig=figure(2);
fig.Position=[200 0 1800 1400];
% Kp_range=gain_maxes(1)-gain_mins(1);
% resol=20;
% Kp_surf_resol=Kp_range/resol;
% Ki_range=gain_maxes(2)-gain_mins(2);
% Ki_surf_resol=Ki_range/resol;
% [kp_pt,ki_pt]=meshgrid(gain_mins(1):Kp_surf_resol:gain_maxes(1),gain_mins(2):Ki_surf_resol:gain_maxes(2));
% j_pt=zeros(size(kp_pt));
% c_pt=zeros(size(kp_pt));
% for i=1:size(kp_pt,1)
%     for j=1:size(kp_pt,2)
%         [l,c]=ObjFun([kp_pt(i,j),ki_pt(i,j)],G);
%         j_pt(i,j)=l;
%         c_pt(i,j)=c;
%     end
% end
% j_pt(c_pt>0.0)=NaN;
ax1 = axes; hold on;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
surf(ax1, kp_pt,ki_pt,reshape(j_pt,size(kp_pt)),'EdgeColor','Interp','FaceColor','Interp');
ms_true=[0.5464, 1.1617];
htrue=plot3(ax1,ms_true(1), ms_true(2), max(j_pt,[],'all'),'p', 'MarkerFaceColor', [1,1,0], 'MarkerSize',20);
xlabel(ax1,'Kp', 'FontSize',24, 'FontName','Times new Roman')
ylabel(ax1,'Ki', 'FontSize',24, 'FontName','Times new Roman')
zlabel(ax1,'J', 'FontSize',24, 'FontName','Times new Roman')
view(ax1,[0,0,1])
xlim(ax1, [min(kp_pt,[],'all'), max(kp_pt,[],'all')])
ylim(ax1, [min(ki_pt,[],'all'), max(ki_pt,[],'all')])
ax2 = axes;
% Hide the top axes
ax2.Visible = 'off';
ax2.XTick = [];
ax2.YTick = [];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
hold on;
% plot mean of sampled gains over all experiments
% meanExperGBO=[];
% meanExperBO=[];
% for i=1:min(length(TraceGBO),length(TraceBO))
%     meanExperGBO(:,:,i)=TraceGBO(i).samples(1:end, :);
%     meanExperBO(:,:,i)=TraceBO(i).samples(1:end, :);
% end
% meanExperGBO=mean(meanExperGBO,3);
% meanExperBO=mean(meanExperBO,3);
hInit=plot3(ax2, Trace(exper).samples(1,1), Trace(exper).samples(1,2),max(j_pt,[],'all'),'x', 'MarkerEdgeColor', [0,0,0] , 'MarkerSize',20);
c = linspace(1,10,length(kp)); 
hGBO=scatter3(ax2, kp, ki, z, [120],c,'filled');
% hBO=scatter3(ax2, meanExperBO(N0+1:end, 1), meanExperBO(N0+1:end, 2), ones(length(meanExperGBO(N0+1:end,1)), 1), [120],c,'filled','^');

colormap(ax1, flipud(parula));
colormap(ax2,flipud(copper));
set(ax1,'ColorScale','log')
% get everthin lined up
cb1 = colorbar(ax1,'Position',[0.05 0.11 0.01 0.815]); % four-elements vector to specify Position [left bottom width height]
cb2 = colorbar(ax2,'Position',[0.92 0.11 0.01 0.815]);
cb1.Label.String = 'Cost';
cb1.FontName='Times New Roman';
cb1.FontSize=24;
cb2.Label.String = 'iteration';
cb2.FontName='Times New Roman';
cb2.FontSize=24;
legend([hGBO, htrue, hInit],{'Guided BO', 'BO', 'Ground Truth', 'Initial Sample'}, 'Location', 'north', 'fontname','Times New Roman');
grid on

set(gca, 'DefaultAxesFontName', 'Times', 'FontSize', 24), 
