function rup_kel_1
clear all; clc; close all;
% % KUKA LBR IIWA
% Jl=5.6;
% bc=55;
% Jc=1.03;
% K=18500;
% Td=5;
% G = tf([Jl*bc, Jl*K],[Jc*Jl, Jl*bc, Jc*K+Jl*K],'InputDelay',Td);

% ball-screw system
Kcp=60;
Kci=1000;
Kcd=18;
Ra=9.02;
La=0.0187;
Kt=0.515;
Kb=0.55;
Jm=0.27e-4;
Bm=0.0074;
Jl=6.53e-4;
Bml=0.014;
Ks=3e7;
G=tf([Kt],[La*(Jm+Jl),La*Bm+Ra*(Jm+Jl),Ra*Bm+Kt*Kb])*tf([Bml,Ks],[Jl,Bml,Ks]);


% % auto tune
% C_tuned = pidtune(G,'PID')
% Kdc=C_tuned.Kd;
% Kpc=C_tuned.Kp;
% Kic=C_tuned.Ki;

Kpc=1.;
Kic=100.;
Kdc=0.;

% search_span_d=100;
search_span_p=2.;
search_span_i=20.;
search_span_d=.2;

Kp_min=Kpc-search_span_p/2;
Kp_max=Kpc+search_span_p/2;
Ki_min=Kic-search_span_i/2;
Ki_max=Kic+search_span_i/2;
Kd_min=Kdc-search_span_d/2;
Kd_max=Kdc+search_span_d/2;

% initial values for GP of BO
N0=10;
Kp0 = (Kp_max-Kp_min).*rand(N0,1) + Kp_min;
Ki0 = (Ki_max-Ki_min).*rand(N0,1) + Ki_min;
Kd0 = (Kd_max-Kd_min).*rand(N0,1) + Kd_min;
sampleTf=1;
sampleTs=sampleTf/(10-1);
Tf=1e-8;
for i=1:N0
    C=tf([0+Tf*Kp0(i),Kp0(i)+Tf*Ki0(i),Ki0(i)], [Tf, 1, 0]);
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
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    if i==1
        data = iddata(ytmp,utmp,sampleTs);
    else
        data = merge(data, iddata(ytmp,utmp,sampleTs));
    end
end

% surrogate model
np2=2;
G2 = tfest(data,np2);

InitData=table(Kp0, Ki0, Kd0);

Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');
Kd = optimizableVariable('Kd', [Kd_min Kd_max], 'Type','real');

vars=[Kp, Ki, Kd];
fun = @(vars)myObjfun_withApproximateModel(vars, G, G2, Tf, sampleTf, sampleTs, np2, data);
% fun = @(vars)myObjfun_withoutApproximateModel(vars, G, Tf);
FileName='demo_6/results.mat';
results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 100, 'NumSeedPoints', N0, ...
    'PlotFcn', 'all', 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound', 'OutputFcn', @saveToFile, 'SaveFileName', append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', FileName));

% FinalBestResult = bestPoint(results)
end

function [objective] = myObjfun_withApproximateModel(vars, G, G2, Tf, sampleTf, sampleTs, np2, data)
persistent N
persistent idx

if isempty(N)
    N=1;
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    if abs(stepinfo(CL).Overshoot)<0.01
        objective = abs(0.01*stepinfo(CL).SettlingTime);
    else
        objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    end
    idx= 0;
elseif idx==10
    G2 = tfest(data,np2);
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    if abs(stepinfo(CL).Overshoot)<0.01
        objective = abs(0.01*stepinfo(CL).SettlingTime);
    else
        objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    end
    idx= 0;
else
    %     todo move some lines outside with handler@: faster?
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    if abs(stepinfo(CL).Overshoot)<0.01
        objective = abs(0.01*stepinfo(CL).SettlingTime);
    else
        objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    end
    
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
N = N+1;
end


function [objective] = myObjfun_withoutApproximateModel(vars, G, Tf)
%     todo move some lines outside with handler@: faster?
C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
CL=feedback(C*G, 1);
if abs(stepinfo(CL).Overshoot)<0.01
    objective = abs(0.01*stepinfo(CL).SettlingTime);
else
    objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
end
end



