close all
clear all
clc
load('debug_G2data_1.mat')
load('debug_G2_1_3.mat')
npG2=2;
nzG2=1;
Options = tfestOptions('Display','off');
Options.InitialCondition = 'backcast';
Options.EnforceStability=1;
G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
% hyper_cand=[0.500000000000000	1.47000000000000];
hyper_cand=[0.329000000000000	0.870733221719789];
% hyper_cand=[0.329000000000000	0.870733221719789];
% hyper_cand=[0.862085238159328	1.09324049805123];
% hyper_cand=[0.934624062654655	0.969098216469554];
% hyper_cand=[0.329000000000000	0.870733221719789];
Kp = hyper_cand(1);
Ti = 1/hyper_cand(2);
Td = 0; 
N=inf;
Ts = 0.01;
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
t2=Ts.*(0:length(cell2mat(G2data.y(2)))-1);
plot(t2,cell2mat(G2data.y(2)),'b')
legend(["surrogate estimate", "measured"])



npG2=2;
nzG2=1;
Options = tfestOptions('Display','off');
Options.InitialCondition = 'backcast';
Options.EnforceStability=1;
G2check = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
