close all;
clc;
clear;
idName= 'results_1';
dir=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_72_eta2_02_eta1_3/results_1/');
N0=1; %number of initial data
N_iter=50;
N_iter=N_iter+N0;
% todo automatize code
load(append(dir,'trace_file.mat'),'Trace')
TraceGBO=Trace;
clear Trace
load(append(dir,'trace_file_BO.mat'),'Trace')
TraceBO=Trace;
clear Trace
dir_gains=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds/KpKi_bounds_new_2.mat');
load(dir_gains, 'Kp_min', 'Kp_max', 'Ki_min', 'Ki_max')
gain_mins=[Kp_min, Ki_min];
gain_maxes=[Kp_max, Ki_max];
% DC motor at FHNW lab
% speed sensor pole 9.918e-5
num = [9.54434];
den = [1, 4.14479, 4.19941];
Td=2e-3;
G = tf(num, den, 'InputDelay',Td);
Kp_range=gain_maxes(1)-gain_mins(1);
resol=20;
Kp_surf_resol=Kp_range/resol;
Ki_range=gain_maxes(2)-gain_mins(2);
Ki_surf_resol=Ki_range/resol;
[kp_pt,ki_pt]=meshgrid(gain_mins(1):Kp_surf_resol:gain_maxes(1),gain_mins(2):Ki_surf_resol:gain_maxes(2));
j_pt=zeros(size(kp_pt));
c_pt=zeros(size(kp_pt));
for i=1:size(kp_pt,1)
    for j=1:size(kp_pt,2)
        [l,c]=ObjFun([kp_pt(i,j),ki_pt(i,j)],G, false);
        j_pt(i,j)=l;
        c_pt(i,j)=c;
    end
end
j_pt(c_pt>0.0)=NaN;

exper=26
fig=figure(exper);
% title(string(exper))
fig.Position=[200 0 1800 800];
ax1 = axes; hold on;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
surf(ax1, kp_pt,ki_pt,reshape(j_pt,size(kp_pt)),'EdgeColor','Interp','FaceColor','Interp');
xlabel(ax1,'Kp', 'FontSize',24, 'FontName','Times new Roman')
ylabel(ax1,'Ki', 'FontSize',24, 'FontName','Times new Roman')
zlabel(ax1,'J', 'FontSize',24, 'FontName','Times new Roman')
view(ax1,[0,0,1])
xlim(ax1, [gain_mins(1), gain_maxes(1)])
ylim(ax1, [gain_mins(2), gain_maxes(2)])


ax2 = axes;
% Hide the top axes
ax2.Visible = 'off';
ax2.XTick = [];
ax2.YTick = [];
% plot metrics=[emax, Ts, Tr, ITAE, ess, ov] vs iteration
hold on;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
ax2.FontSize=24;
ax2.FontName='Times New Roman';
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)

ms_true=[0.5464, 1.1617];
ms_nominal=[0.854, 1.0714];
c = linspace(1,length(TraceGBO(exper).values(N0+1:end,1)),length(TraceGBO(exper).values(N0+1:end,1)));
hGBO=plot3(ax2, TraceGBO(exper).samples(N0+1:end, 1), TraceGBO(exper).samples(N0+1:end, 2), TraceGBO(exper).values(N0+1:end, 1),'o', 'MarkerEdgeColor', 'black' , 'MarkerSize',25,'MarkerFaceColor','red');
hBO=plot3(ax2, TraceBO(exper).samples(N0+1:end, 1), TraceBO(exper).samples(N0+1:end, 2), TraceBO(exper).values(N0+1:end, 1),'^', 'MarkerEdgeColor', 'black' , 'MarkerSize',25, 'MarkerFaceColor','cyan');
htrue=plot3(ax2,ms_true(1), ms_true(2),2.4,'pentagram', 'MarkerFaceColor', [0,1,0], 'MarkerEdgeColor', 'black', 'MarkerSize',25);
hnom=plot3(ax2,ms_nominal(1), ms_nominal(2), 0.9915+.5,'hexagram', 'MarkerFaceColor', [1,1,0], 'MarkerEdgeColor', 'black', 'MarkerSize',25);
hinit=plot3(ax2, TraceGBO(exper).samples(1:N0, 1), TraceGBO(exper).samples(1:N0, 2), TraceGBO(exper).values(1:N0, 1),'square', 'MarkerEdgeColor', 'black' , 'MarkerSize',35, 'MarkerFaceColor','black');

% hGBO=scatter3(ax2, meanExperGBO(N0+1:end, 1), meanExperGBO(N0+1:end, 2), ones(length(meanExperGBO(N0+1:end,1)), 1), [100],[0,0,0],'filled');
% hBO=scatter3(ax2, meanExperBO(N0+1:end, 1), meanExperBO(N0+1:end, 2), ones(length(meanExperGBO(N0+1:end,1)), 1), [100],[0,0,0],'filled','^');

% plot for specific experiment
% plot3(ax2, TraceGBO.samples(1:N0, 1), TraceGBO.samples(1:N0, 2), TraceGBO.values(1:N0, 1),'x', 'MarkerFaceColor', [0,0,0]);
% c = linspace(1,length(TraceGBO.values(N0+1:end,1)),length(TraceGBO.values(N0+1:end,1)));
% hGBO=scatter3(ax2, TraceGBO.samples(N0+1:end, 1), TraceGBO.samples(N0+1:end, 2), TraceGBO.values(N0+1:end, 1), [100],c,'filled');
% hBO=scatter3(ax2, TraceBO.samples(N0+1:end, 1), TraceBO.samples(N0+1:end, 2), TraceBO.values(N0+1:end, 1), [100],c,'filled','^');
% Give each one its colormap
colormap(ax1,flipud(parula));
% colormap(ax2,'hot');
caxis(ax1, [.5 2.5])
set(ax1,'ColorScale','log')
% get everthin lined up
cb1 = colorbar(ax1,'Position',[0.92 0.11 0.01 0.815]); % four-elements vector to specify Position [left bottom width height]
% cb2 = colorbar(ax2,'Position',[0.915 0.11 0.01 0.815]);
cb1.Label.String = 'Cost';
% cb2.Label.String = 'iteration';
% legend([hGBO, hBO, hinit],{'Guided BO', 'BO', 'Initial data'}, 'Location', 'northwest');
legend([hinit, htrue, hnom, hBO, hGBO],{'Initial gains', 'Ground Truth', 'nominal gains', 'BO', 'Guided BO',}, 'Position', [0.79,0.68,0.1,0.1], 'fontname','Times New Roman');
grid on

set(gca, 'DefaultAxesFontName', 'Times')
figName=append(dir, idName,'_KpKiCost_expr72_eta1_3_exper26.png');
print(gcf,figName,'-dpng','-r300');
saveas(gcf,figName)
figName=append(dir, idName,'_KpKiCost_expr72_eta1_3_exper26.fig');
saveas(gcf,figName)

function [objective, constraints] = ObjFun(X, G, objective_noise)
C=tf([X(1), X(1)*X(2)], [1, 0]);
CL=feedback(C*G, 1);
STPinfo=stepinfo(CL, 'RiseTimeLimits',[0.1,0.6]);
ov=abs(STPinfo.Overshoot);
st=STPinfo.SettlingTime;
[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=STPinfo.RiseTime;
ITAE = trapz(t, t.*abs(e));
if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end

if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end
if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=1e5;
end
if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=1e5;
end
w_mean_grid=[10.5360, 3.8150, 0.6119, 1.1596];
w_importance=[2, 1, 1, 1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);
if objective_noise==true
    noise=0.0035*randn(1,1);
    objective=objective+noise;
end
constraints=-1;
end
