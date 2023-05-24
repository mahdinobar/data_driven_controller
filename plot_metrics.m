close all; clear;clc
fig=figure(1);
fig.Position=[200 0 1600 1000];
ax1=axes;
ax1.FontSize=24;
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
