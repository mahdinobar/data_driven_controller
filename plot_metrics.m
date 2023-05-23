close all; clear;clc
fig=figure(1);
fig.Position=[200 0 800 600];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on
sys = tf(4,[1 2 10]);
[y,NA] = step(sys);
h1=plot(y+0.1,"-b","LineWidth",2);
xlim([-20,125])
ylim([0.05,0.7])
t=-20:0.005:125;
r=t>0;
r=r.*0.401+0.1;
h2=plot(t,r,'-r',"LineWidth",2);
rlim_up=(0.401+0.1)*102/100;
rlim_down=(0.401+0.1)*98/100;
h3=yline(rlim_up,'--k',"LineWidth",1);
h4=yline(rlim_down,'--k',"LineWidth",1);
box on
set(gca,'xtick',[])
set(gca,'ytick',[])
legend([h1,h2],["measured output","reference input"],"Location","southeast")
% ylabel("y")
% xlabel("time(t)")
saveas("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/metrics_visualization.png")
