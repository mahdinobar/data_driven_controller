function rup_kel_1
clear all; clc; close all;

q6u6=load("/home/mahdi/RLI/codes/pbdlib-sandbox-matlab/demos/tmp/rupenyan/q6u6.mat");
Ts=0.05;
nbData_interp=100*4;
Ts_interp=Ts/4;
noise1= 0.1*randn(1,nbData_interp);
q_interp=linspace(min(q6u6.q),max(q6u6.q),nbData_interp);
u_interp = interp1(q6u6.q,q6u6.u,q_interp)+noise1;
noise2= 0.1*randn(1,nbData_interp);
q_interp=q_interp+noise2;
data = iddata(q_interp',u_interp',Ts_interp);

% full model
np1=8;
G1 = tfest(data,np1);

% surrogate model
np2=2;
G2 = tfest(data,np2);

% select the model
G=G1;

% auto tune
% C_tuned = pidtune(G,'PID');

Kdc=0.;
Kpc=0.;
Kic=0.;
% uncomment to warmstart G8 optimization centered at optimum results for G2
% Kdc=0.28188;
% Kpc=0.46797;
% Kic=0.00031961;

Kd_min=Kdc-.5;
Kd_max=Kdc+.5;
Kp_min=Kpc-.5;
Kp_max=Kpc+.5;
Ki_min=Kic-.5;
Ki_max=Kic+.5;

Kd = optimizableVariable('Kd', [Kd_min Kd_max], 'Type','real');
Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');

epsilon=1e-3;
vars=[Kp, Ki, Kd];
fun = @(vars)myObjfun(vars, G, epsilon);
results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 100); 


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

function [objective] = myObjfun(vars, G, epsilon)
%     todo move some lines outside with handler@: faster?
C=tf([vars.Kd,vars.Kp,vars.Ki], [1, 0]);
CL=feedback(C*G, 1);

objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime)+epsilon;
end


