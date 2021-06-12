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

% initial values for GP of BO
N0=10;
Kd0 = (Kd_max-Kd_min).*rand(N0,1) + Kd_min;
Kp0 = (Kp_max-Kp_min).*rand(N0,1) + Kp_min;
Ki0 = (Ki_max-Ki_min).*rand(N0,1) + Ki_min;
uSurrogate=[];
ySurrogate=[];
sampleT=1;
Tf=1e-4;
for i=1:N0
    C=tf([Kd0(i)+Tf*Kp0(i),Kp0(i)+Tf*Ki0(i),Ki0(i)], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    while isnan(objective)
        Kd0(i) = (Kd_max-Kd_min).*rand(1,1) + Kd_min;
        Kp0(i) = (Kp_max-Kp_min).*rand(1,1) + Kp_min;
        Ki0(i) = (Ki_max-Ki_min).*rand(1,1) + Ki_min;
        C=tf([Kd0(i)+Tf*Kp0(i),Kp0(i)+Tf*Ki0(i),Ki0(i)], [Tf, 1, 0]);
        CL=feedback(C*G, 1);
        objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    end
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleT);
    utmp=step(CLU,0:sampleT);
    ySurrogate=[ySurrogate,ytmp(2)];
    uSurrogate=[uSurrogate,utmp(2)];
end
InitData=table(Kd0, Kp0, Ki0);


Kd = optimizableVariable('Kd', [Kd_min Kd_max], 'Type','real');
Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');

vars=[Kp, Ki, Kd];
fun = @(vars)myObjfun(vars, G, Tf);
results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 100, 'NumSeedPoints', N0, 'InitialX', InitData);


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

function [objective] = myObjfun(vars, G, Tf)
persistent N
if isempty(N)
    N=1;
end
%     todo move some lines outside with handler@: faster?
C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
CL=feedback(C*G, 1);

objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);

N = N+1;
end



