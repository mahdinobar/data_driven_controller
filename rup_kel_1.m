function rup_kel_1
clear all; clc; close all;

q6u6=load("/home/mahdi/RLI/codes/pbdlib-sandbox-matlab/demos/tmp/rupenyan/q6u6.mat");
Ts=0.05;
nbData_interp=100*4;
Ts_interp=Ts/4;
q_interp=linspace(min(q6u6.q),max(q6u6.q),nbData_interp);
u_interp = interp1(q6u6.q,q6u6.u,q_interp);
data = iddata(q_interp',u_interp',Ts_interp);
np1=8;
G1 = tfest(data,np1);
% C_tuned = pidtune(G1,'PID');






Kd = optimizableVariable('Kd', [-1 1], 'Type','real');
Kp = optimizableVariable('Kp', [-1 1], 'Type','real');
Ki = optimizableVariable('Ki', [-1 1], 'Type','real');

vars=[Kp, Ki, Kd];
fun = @(vars)myObjfun(vars, G1);
results = bayesopt(fun,vars); 

% np2=2;
% G2 = tfest(data,np2);

% load carbig
% X = [Acceleration Cylinders Displacement Weight];
% Y = MPG;
%
% sys = tf([1 5 5],[1 1.65 5 6.5 2]);
% S = stepinfo(sys)
%
% openExample('ident/EstimateTransferFunctionModelExample')
% load iddata1 z1;
end

function [objective] = myObjfun(vars, G1)
%     todo move some lines outside with handler@: faster?

C=tf([vars.Kd,vars.Kp,vars.Ki], [1, 0]);
CL=feedback(C*G1, 1);
objective = stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime;
% J=stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime;
end


