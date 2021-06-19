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
Tf=1e-8;

% % uncomment to estimate stable gain bounds
% % auto tune
% C_tuned = pidtune(G,'PID');
% Kd_nominal=C_tuned.Kd;
% Kp_nominal=C_tuned.Kp;
% Ki_nominal=C_tuned.Ki;

% max_overshoot=0;
% d=1e-3;
% while max_overshoot<9 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-d, Ki_nominal-d, Kd_nominal-d];
%     ub=[Kp_nominal+d, Ki_nominal+d, Kd_nominal+d];
%     funPS_handle = @(x)funPS(x, G, Tf);
%     x = particleswarm(funPS_handle,3,lb,ub);
% 
%     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     d = d*1.5;
% end
%     function [objective] = funPS(x, G, Tf)
%         %     todo move some lines outside with handler@: faster?
%         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%         CL=feedback(C*G, 1);
%         objective=-abs(stepinfo(CL).Overshoot);
%         if isnan(objective)
%             objective=-inf;
%         end
%     end
% d=d/1.5;
% Kp_min=Kp_nominal-d;
% Kp_max=Kp_nominal+d;
% Ki_min=Ki_nominal-d;
% Ki_max=Ki_nominal+d;
% Kd_min=Kd_nominal-d;
% Kd_max=Kd_nominal+d;

% max_overshoot=0;
% dp=1e-2;
% di=1e0;
% dd=1e-3;
% while max_overshoot<25 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
%     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
%     funPS_handle = @(x)funPS(x, G, Tf);
%     x = particleswarm(funPS_handle,3,lb,ub);
% 
%     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     dp = dp*1.5
% end
% dp = dp/1.5;
% max_overshoot=0;
% while max_overshoot<50 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
%     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
%     funPS_handle = @(x)funPS(x, G, Tf);
%     x = particleswarm(funPS_handle,3,lb,ub);
% 
%     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     di = di*1.5
% end
% di = di/1.5;
% max_overshoot=0;
% while max_overshoot<75 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
%     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
%     funPS_handle = @(x)funPS(x, G, Tf);
%     x = particleswarm(funPS_handle,3,lb,ub);
% 
%     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     dd = dd*1.5
% end
% dd = dd/1.5;
%     function [objective] = funPS(x, G, Tf)
%         %     todo move some lines outside with handler@: faster?
%         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%         CL=feedback(C*G, 1);
%         objective=-abs(stepinfo(CL).Overshoot);
%         if isnan(objective)
%             objective=-inf;
%         end
%     end
% 
% Kp_min=Kp_nominal-dp;
% Kp_max=Kp_nominal+dp;
% Ki_min=Ki_nominal-di;
% Ki_max=Ki_nominal+di;
% Kd_min=Kd_nominal-dd;
% Kd_max=Kd_nominal+dd;
% 
% save('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/ball_screw_gain_bounds/KpKiKd_bounds.mat','Kp_min','Ki_min','Kd_min', 'Kp_max','Ki_max','Kd_max')

load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/ball_screw_gain_bounds/KpKiKd_bounds.mat')

% Kpc=1.;
% Kic=100.;
% Kdc=0.;
% % search_span_d=100;
% search_span_p=2.;
% search_span_i=20.;
% search_span_d=.2;
% Kp_min=Kpc-search_span_p/2;
% Kp_max=Kpc+search_span_p/2;
% Ki_min=Kic-search_span_i/2;
% Ki_max=Kic+search_span_i/2;
% Kd_min=Kdc-search_span_d/2;
% Kd_max=Kdc+search_span_d/2;


% add extra safety margin
safeFac=1e-3;
rgKp=(Kp_max-Kp_min);
Kp_min=Kp_min+safeFac*rgKp;
Kp_max=Kp_max-safeFac*rgKp;
rgKi=(Ki_max-Ki_min);
Ki_min=Ki_min+safeFac*rgKi;
Ki_max=Ki_max-safeFac*rgKi;
rgKd=(Kd_max-Kd_min);
Kd_min=Kd_min+safeFac*rgKd;
Kd_max=Kd_max-safeFac*rgKd;

% initial values for GP of BO
N0=10;
Kp = (Kp_max-Kp_min).*rand(N0,1) + Kp_min;
Ki = (Ki_max-Ki_min).*rand(N0,1) + Ki_min;
Kd = (Kd_max-Kd_min).*rand(N0,1) + Kd_min;
objectiveData = zeros(N0,1);
sampleTf=0.07;
sampleTs=sampleTf/(10-1);
for i=1:N0
    C=tf([Kd(i)+Tf*Kp(i),Kp(i)+Tf*Ki(i),Ki(i)], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    objectiveData(i) = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    while isnan(objectiveData(i))
        Kd(i) = (Kd_max-Kd_min).*rand(1,1) + Kd_min;
        Kp(i) = (Kp_max-Kp_min).*rand(1,1) + Kp_min;
        Ki(i) = (Ki_max-Ki_min).*rand(1,1) + Ki_min;
        C=tf([Kd(i)+Tf*Kp(i),Kp(i)+Tf*Ki(i),Ki(i)], [Tf, 1, 0]);
        CL=feedback(C*G, 1);
        objectiveData(i) = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
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

InitData=table(Kp, Ki, Kd);

Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');
Kd = optimizableVariable('Kd', [Kd_min Kd_max], 'Type','real');

vars=[Kp, Ki, Kd];
% fun = @(vars)myObjfun_withApproximateModel(vars, G, G2, Tf, sampleTf, sampleTs, np2, data);
% fun = @(vars)myObjfun_withoutApproximateModel(vars, G, Tf);
fun = @(vars)myObjfun_ApproxLoop(vars, G, G2, Tf, sampleTf, sampleTs, np2, data);

N_iter=100;
idx=0;
global N
for iter=N0:N_iter
    iter
    results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', N0+1, 'NumSeedPoints', N0, ...
            'PlotFcn', {}, 'InitialObjective', objectiveData, 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound');
    nanCheck = results.MinObjective;
    while isnan(nanCheck)
        results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', N0+1, 'NumSeedPoints', N0, ...
            'PlotFcn', {}, 'InitialObjective', objectiveData, 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound');
        nanCheck = results.MinObjective;
        N=N-1;
    end
    InitData=[InitData; results.XAtMinObjective];
    objectiveData = [objectiveData; results.MinObjective];
    
    N0=N0+1;
%     uncomment for surrogate model
%     remove previos data of older surrogate model
    if rem(N0,11)==0 && N0>11
        idx=idx+1;
        InitData([N0-11],:)=[];
        objectiveData([N0-11],:)=[];
    end
    counter=N0-idx;
        
    
    %     FileName='results.mat';
    %     results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 1, 'NumSeedPoints', N0, ...
    %         'PlotFcn', 'all', 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound', 'OutputFcn', @saveToFile, 'SaveFileName', append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', FileName));
end

plot(objectiveData)
pause;
% FinalBestResult = bestPoint(results)
end


function [objective] = myObjfun_ApproxLoop(vars, G, G2, Tf, sampleTf, sampleTs, np2, data)
global N
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
%     if isnan(objective)
%         objective=1e5;
%     end
%     objective=max(objective, 1e5);
    
    
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
N = N+1;
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
    if isnan(objective)
        objective=1e5;
    end
    objective=max(objective, 1e5);
    
    
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



