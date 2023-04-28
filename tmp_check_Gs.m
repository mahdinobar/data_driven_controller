close all
clear all
clc
load('debug_G2data_10.mat')
load('debug_G2_10_12.mat')
load('trace_file_removed.mat')
npG2=2;
nzG2=1;
Options = tfestOptions('Display','off');
Options.InitialCondition = 'backcast';
Options.EnforceStability=1;
Ts = 0.01;
G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', Ts);

exper=7;
hyper_cand=Trace.samples(exper,:);
Kp = hyper_cand(1);
Ti = 1/hyper_cand(2);
Td = 0; 
N=inf;
C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
CL=feedback(C*G2, 1);
reference0=0;
reference=40;
t_high=(11*Ts):Ts:(5.1-Ts);
t_low=0:Ts:(10*Ts);
step_high=reference.*ones(length(t_high),1);
step_low=reference0.*ones(length(t_low),1);
t=[t_low,t_high]';
r=[step_low;step_high];
y2=lsim(CL,r,t);

figure(1)
plot(t,y2,'r');
hold on
t2=Ts.*(0:length(cell2mat(G2data.y(exper)))-1);
plot(t2,cell2mat(G2data.y(exper)),'b')
legend(["surrogate estimate", "measured"])



npG2=2;
nzG2=1;
Options = tfestOptions('Display','off');
Options.InitialCondition = 'backcast';
Options.EnforceStability=1;
G2check = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
