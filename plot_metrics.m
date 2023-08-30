close all; clear;clc
fig=figure(1);
fig.Position=[200 0 1600 700];
ax1=axes;
ax1.FontSize=32;
ax1.FontName='Times New Roman';
hold on
sys = tf(4,[1 2 10]);
[y,NA] = step(sys);
h1=plot(y+0.1,"-b","LineWidth",1.5);
xlim([-20,125])
ylim([0.05,0.7])
t=-20:0.001:125;
r=t>0.6;
r=r.*0.401+0.1;
h2=plot(t,r,'-r',"LineWidth",1.5);
rlim_up=(0.401+0.1)*102/100;
rlim_down=(0.401+0.1)*98/100;
% h3=yline(rlim_up,'--k',"LineWidth",1);
% h4=yline(rlim_down,'--k',"LineWidth",1);
legend([h1,h2],["measured output","reference input"],"Location","northeast")
box on
set(gca,'xtick',[])
set(gca,'ytick',[])
% ylabel("y")
% xlabel("time(t)")
saveas(gca,"/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/metrics_visualization_2.png")
%%

fig=figure(2);
fig.Position=[200 0 1600 300];
ax2=axes;
ax2.FontSize=32;
ax2.FontName='Times New Roman';
hold on
sys = tf(4,[1 2 10]);
[y,tOut] = step(sys);
yy=[zeros(21,1);y+0.1-0.501];
t=linspace(-20,125,length(yy));
h1=plot(t,yy,"-k","LineWidth",1.5);
xlim([-20,125])
ylim([-0.41,0.2])
%
yt=[zeros(20,1);tOut.*(y+0.1-0.501)];
t=linspace(-20,125,length(yt));
h2=plot(t,yt,"-r","LineWidth",1.5);
yline(0,"--");
rlim_up=(0.401+0.1)*102/100;
rlim_down=(0.401+0.1)*98/100;
% h3=yline(rlim_up,'--k',"LineWidth",1);
% h4=yline(rlim_down,'--k',"LineWidth",1);
legend([h1, h2],["$|y(t)-y_{2}|$","$t|y(t)-y_{2}|$"],'Interpreter','latex',"Location","northeast")
box on
set(gca,'xtick',[])
set(gca,'ytick',[])

