function rup_kel_1
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_0_2';
sys='robot_arm';
N0=3;
N_iter=30;
repeat_experiment=2;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir=append(tmp_dir,'/', idName, '/');
% dir=append('/idiap/home/mnobar/Rupenyan/data_driven_controller/tmp/', idName, '/');


% % KUKA LBR IIWA
% Jl=5.6;
% bc=55;
% Jc=1.03;
% K=18500;
% Td=5;
% G = tf([Jl*bc, Jl*K],[Jc*Jl, Jl*bc, Jc*K+Jl*K],'InputDelay',Td);

% % ball-screw system
% Kcp=60;
% Kci=1000;
% Kcd=18;
% Ra=9.02;
% La=0.0187;
% Kt=0.515;
% Kb=0.55;
% Jm=0.27e-4;
% Bm=0.0074;
% Jl=6.53e-4;
% Bml=0.014;
% Ks=3e7;
% G=tf([Kt],[La*(Jm+Jl),La*Bm+Ra*(Jm+Jl),Ra*Bm+Kt*Kb])*tf([Bml,Ks],[Jl,Bml,Ks]);

% robot-arm System ("Simultaneous computation of model order..., Badaruddin Muhammad et al.")
num = [-0.0118, 0.0257, 0, 0, 0];
den = [1, -3.1016, 4.3638, -3.1528, 1.0899, -0.0743];
ts=1;
G = d2c(tf(num,den, ts));

% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

% % uncomment to estimate stable gain bounds
% % auto tune
% C_tuned = pidtune(G,'PID');
% Kd_nominal=C_tuned.Kd;
% Kp_nominal=C_tuned.Kp;
% Ki_nominal=C_tuned.Ki;
% 
% Tf=1e-8;
% 
% % C=tf([Kd_nominal+Tf*Kp_nominal,Kp_nominal+Tf*Ki_nominal,Ki_nominal], [Tf, 1, 0]);
% % CL=feedback(C*G, 1);
% % if abs(stepinfo(CL).Overshoot)<1
% %     objective = abs(1*stepinfo(CL).SettlingTime);
% % else
% %     objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
% % end
% % objective
% 
% 
% max_overshoot=0;
% d=1e-1;
% while max_overshoot<50 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-d, Ki_nominal-d, Kd_nominal-d];
%     ub=[Kp_nominal+d, Ki_nominal+d, Kd_nominal+d];
%     funPS_handle = @(x)funPS(x, G, Tf);
%     x = particleswarm(funPS_handle,3,lb,ub);
% 
%     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     d
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     d = d*1.1;
% end
%     function [objective] = funPS(x, G, Tf)
%         %     todo move some lines outside with handler@: faster?
%         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%         CL=feedback(C*G, 1);
%         objective=-abs(stepinfo(CL).Overshoot);
%         if isnan(objective)
%             objective=-1e10;
%         end
%     end
% d=d/1.1;
% Kp_min=Kp_nominal-d;
% Kp_max=Kp_nominal+d;
% Ki_min=Ki_nominal-d;
% Ki_max=Ki_nominal+d;
% Kd_min=Kd_nominal-d;
% Kd_max=Kd_nominal+d;
% 
% % max_overshoot=0;
% % dp=1e-1;
% % di=1e-6;
% % dd=1e-6;
% % while max_overshoot<25 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     dp
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     dp = dp*1.5;
% % end
% % dp = dp/1.5;
% % max_overshoot=0;
% % while max_overshoot<50 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     di
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     di = di*1.5;
% % end
% % di = di/1.5;
% % max_overshoot=0;
% % while max_overshoot<75 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     dd
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     dd = dd*1.5;
% % end
% % dd = dd/1.5;
% %     function [objective] = funPS(x, G, Tf)
% %         %     todo move some lines outside with handler@: faster?
% %         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %         CL=feedback(C*G, 1);
% %         objective=-abs(stepinfo(CL).Overshoot);
% %         if isnan(objective)
% %             objective=-1e10;
% %         end
% %     end
% %
% % Kp_min=Kp_nominal-dp;
% % Kp_max=Kp_nominal+dp;
% % Ki_min=Ki_nominal-di;
% % Ki_max=Ki_nominal+di;
% % Kd_min=Kd_nominal-dd;
% % Kd_max=Kd_nominal+dd;
% % save('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/robot_arm_gain_bounds/KpKiKd_bounds.mat','Kp_min','Ki_min','Kd_min', 'Kp_max','Ki_max','Kd_max')
% save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds.mat','Kp_min','Ki_min','Kd_min', 'Kp_max','Ki_max','Kd_max')
if sys=="ball_screw"
    dir_gains=append(tmp_dir,'/', 'ball_screw_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="robot_arm"
    dir_gains=append(tmp_dir,'/', 'robot_arm_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'KpKiKd_bounds.mat');
end
load(dir_gains)

% % % % Find True values
% % % lb=[-0.0954016552295164*1.01,0.186083733952419*0.99,-0.0954016552295164*1.01];
% % % ub=[-0.0954016552295164*0.99,0.186083733952419*1.01,-0.0954016552295164*0.99];
% lb=[-0.2358, 0.0457, -0.2358];
% ub=[0.2358, 0.5173, 0.2358];
% funPS_handle = @(x)funPS2(x, G, Tf);
% % options = optimoptions('particleswarm','FunctionTolerance', 0, 'MaxIterations', 1e3, 'MaxStallIterations', 1e2, 'ObjectiveLimit', 0);
% x = particleswarm(funPS_handle,3,lb,ub);
% % % x.Kp=0.231167276750434;
% % % x.Ki=0.467605534040020;
% % % x.Kd=0.234584924311749;
% True_objective=funPS2(x, G, Tf)
%     function [objective] = funPS2(x, G, Tf)
%         %         todo move some lines outside with handler@: faster?
%         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%         CL=feedback(C*G, 1);
%         ov=abs(stepinfo(CL).Overshoot);
%         st=stepinfo(CL).SettlingTime;
%         if isnan(ov) || isinf(ov) || ov>1e3
%             ov=1e3;
%         end
%         if isnan(st) || isinf(st) || st>1e5
%             st=1e5;
%         end
%
%             w1=1;     w2=500;     objective=ov/w1+st/w2;
%     end

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


% % add extra safety margin
% safeFacp=1e-1;
% safeFaci=1e-1;
% safeFacd=1e-1;
% rgKp=(Kp_max-Kp_min);
% Kp_min=Kp_min+safeFacp*rgKp;
% Kp_max=Kp_max-safeFacp*rgKp;
% rgKi=(Ki_max-Ki_min);
% Ki_min=Ki_min+safeFaci*rgKi;
% Ki_max=Ki_max-safeFaci*rgKi;
% rgKd=(Kd_max-Kd_min);
% Kd_min=Kd_min+safeFacd*rgKd;
% Kd_max=Kd_max-safeFacd*rgKd;

tmp=[];
% % initial values for GP of BO
RAND=rand(N0,1);

% load(append(dir,'RAND.mat'))

Kp = (Kp_max-Kp_min).*RAND + Kp_min;
Ki = (Ki_max-Ki_min).*RAND + Ki_min;
Kd = (Kd_max-Kd_min).*RAND + Kd_min;
InitobjectiveData = zeros(N0,1);
% todo pay attention how you choose sampleTf?
sampleTf=0.1;
sampleTs=sampleTf/(Nsample-1);
global data
for i=1:N0
    C=tf([Kd(i)+Tf*Kp(i),Kp(i)+Tf*Ki(i),Ki(i)], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    varstmp.Kp=Kp(i);
    varstmp.Ki=Ki(i);
    varstmp.Kd=Kd(i);
    InitobjectiveData(i) = myObjfun_Loop(varstmp, G, Tf);
    while isnan(InitobjectiveData(i)) || InitobjectiveData(i)>1000
        RAND(i)=rand(1,1);
        Kd(i) = (Kd_max-Kd_min).*RAND(i) + Kd_min;
        Kp(i) = (Kp_max-Kp_min).*RAND(i) + Kp_min;
        Ki(i) = (Ki_max-Ki_min).*RAND(i) + Ki_min;
        C=tf([Kd(i)+Tf*Kp(i),Kp(i)+Tf*Ki(i),Ki(i)], [Tf, 1, 0]);
        CL=feedback(C*G, 1);
        varstmp.Kp=Kp(i);
        varstmp.Ki=Ki(i);
        varstmp.Kd=Kd(i);
        InitobjectiveData(i) = myObjfun_Loop(varstmp, G, Tf);
    end
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    %         todo check concept?
    if i==1
        data = iddata(ytmp,utmp,sampleTs);
    else
        data = merge(data, iddata(ytmp,utmp,sampleTs));
    end
end
tmp=[tmp,RAND];
save(append(dir,'RAND_N01.mat'),'tmp')

% surrogate model
% G2tmp = n4sid(data,np2);
% [A,B,C,D,~] = idssdata(G2tmp);
% [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
% G2 = tf(num_surrogate, den_surrogate);

G2 = tfest(data,np2);
% G2_2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.8e0, 0].*(rand(1,1)-0.5));
% G2_3=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
% G2_4=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
% G2_5=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
% G2_6=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
% step(G2, G2_2, G2_3, G2_4, G2_5, G2_6)
InitData=table(Kp, Ki, Kd);

Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');
Kd = optimizableVariable('Kd', [Kd_min Kd_max], 'Type','real');

vars=[Kp, Ki, Kd];
if withSurrogate==true
    if withPerturbed==true
        fun = @(vars)myObjfun_ApproxLoop_perturbed(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat);
    else
        fun = @(vars)myObjfun_ApproxLoop(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat);
    end
else
    fun = @(vars)myObjfun_Loop(vars, G, Tf);
end
objectiveEstData=InitobjectiveData;
XobjectiveEstData=InitData;
objectiveData=InitobjectiveData;
XobjectiveData=InitData;
global N
global idx
N_iter=N_iter+N0;
data_start=data;
InitData_start=InitData;
InitobjectiveData_start=InitobjectiveData;
objectiveEstData_start=objectiveEstData;
XobjectiveEstData_start=XobjectiveEstData;
objectiveData_start=objectiveData;
XobjectiveData_start=XobjectiveData;
InitData_all=InitData;
InitobjectiveData_all=InitobjectiveData;
objectiveEstData_all=objectiveEstData;
XobjectiveEstData_all=XobjectiveEstData;
objectiveData_all=objectiveData;
XobjectiveData_all=XobjectiveData;
for experiment=1:repeat_experiment
    experiment
    
    N=[];
    idx=[];
    counter=N0+1;
    %     objectiveData_not_removed=InitobjectiveData;
    %     objectiveEstData_not_removed=objectiveEstData;
    for iter=N0+1:N_iter
        iter
        %     iteration=iter-N0
        nanCheck=nan;
        %         while isnan(nanCheck)
        results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', counter, 'NumSeedPoints', counter-1, ...
            'PlotFcn', {}, 'InitialObjective', InitobjectiveData, 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound');
        %             nanCheck = results.MinObjective;
        %             error('nanCheck is NAN');
        %         end
        InitData=[InitData; results.NextPoint];
        InitobjectiveData = [InitobjectiveData; myObjfun_Loop(results.NextPoint, G, Tf)];
        objectiveEstData = [objectiveEstData; results.MinEstimatedObjective];
        XobjectiveEstData = [XobjectiveEstData; results.XAtMinEstimatedObjective];
        objectiveData = [objectiveData; results.MinObjective];
        XobjectiveData = [XobjectiveData; results.XAtMinObjective];
        
        %         objectiveData_not_removed=[objectiveData_not_removed; results.MinObjective];
        %         objectiveEstData_not_removed=[objectiveEstData_not_removed; results.MinEstimatedObjective];

        %         remove previos data of older surrogate model
        if withSurrogate==true
            if withPerturbed==true
                %     uncomment for surrogate model
                %     remove previos data of older surrogate model
                if rem(iter-N0,N_real_repeat+(1+num_perturbed_model))==0 && iter>N0+N_real_repeat+(num_perturbed_model)
                    InitData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    InitobjectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    objectiveEstData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    objectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    XobjectiveEstData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    XobjectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1
                    counter=counter-(num_perturbed_model+1);
                end
            else
                if rem(iter-N0-1,N_real_repeat+1)==0 && iter>N0+1
                    InitData([counter-N_real_repeat-1],:)=[];
                    InitobjectiveData([counter-N_real_repeat-1],:)=[];
                    objectiveEstData([counter-N_real_repeat-1],:)=[];
                    objectiveData([counter-N_real_repeat-1],:)=[];
                    XobjectiveEstData([counter-N_real_repeat-1],:)=[];
                    XobjectiveData([counter-N_real_repeat-1],:)=[];
                    counter=counter-1;
                end
            end
        end
        counter=counter+1;
        %     FileName='results.mat';
        %     results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 1, 'NumSeedPoints', N0, ...
        %         'PlotFcn', 'all', 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound', 'OutputFcn', @saveToFile, 'SaveFileName', append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', FileName));
    end
    
    InitData_all=[InitData_all; InitData(N0+1:end,:)];
    InitobjectiveData_all = [InitobjectiveData_all; InitobjectiveData(N0+1:end,:)];
    objectiveEstData_all = [objectiveEstData_all; objectiveEstData(N0+1:end,:)];
    objectiveData_all = [objectiveData_all; objectiveEstData(N0+1:end,:)];
    XobjectiveEstData_all = [XobjectiveEstData_all; XobjectiveEstData(N0+1:end,:)];
    XobjectiveData_all = [XobjectiveData_all; XobjectiveEstData(N0+1:end,:)];
    
    data=data_start;
    InitData=InitData_start;
    InitobjectiveData=InitobjectiveData_start;
    objectiveEstData=objectiveEstData_start;
    objectiveData=objectiveData_start;
    XobjectiveEstData=XobjectiveEstData_start;
    XobjectiveData=XobjectiveData_start;
    
end
save(append(dir,'InitData_all'),'InitData_all')
save(append(dir,'InitobjectiveData_all.mat'),'InitobjectiveData_all')
save(append(dir,'objectiveEstData_all.mat'),'objectiveEstData_all')
save(append(dir,'objectiveData_all.mat'),'objectiveData_all')
save(append(dir,'XobjectiveEstData_all.mat'),'XobjectiveEstData_all')
save(append(dir,'XobjectiveData_all.mat'),'XobjectiveData_all')

% % =========================================================================
% figure(1);hold on
% plot(objectiveData_not_removed(N0+1:end), 'b','DisplayName','MinObjective')
% if withSurrogate==true
%     %     todo
%     for i=1:5
%         xline(i);
%     end
%     for i=31:35
%         xline(i);
%     end
%     %     surrogate_iteration=1:N_real_repeat+1:N_iter-N0;
%     %     for i = 1:size(surrogate_iteration,2)
%     %         xline(surrogate_iteration(i));
%     %     end
% end
% plot(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
% xlabel('iteration')
% ylabel('objective')
% ylim([-0.01 0.01])
% xlim([1 N_iter-N0])
% figName=append(dir, idName, '.png');
% saveas(gcf,figName)
%
% figure(2);hold on
% semilogy(objectiveData_not_removed(N0+1:end), 'b','DisplayName','MinObjective')
% if withSurrogate==true
%     %     todo
%     for i=1:5
%         xline(i);
%     end
%     for i=31:35
%         xline(i);
%     end
%     %     for i = 1:size(surrogate_iteration,2)
%     %         xline(surrogate_iteration(i));
%     %     end
% end
% semilogy(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
% legend('MinObjective','MinEstimatedObjective')
% xlabel('iteration')
% ylabel('objective')
% xlim([1 N_iter-N0])
% figName=append(dir, idName, '_log.png');
% saveas(gcf,figName)
%
% objectiveData_dir=append(dir, idName, '_objectiveData_not_removed.mat');
% save(objectiveData_dir,'objectiveData_not_removed');
% objectiveEstData_dir=append(dir, idName, '_objectiveEstData_not_removed.mat');
% save(objectiveEstData_dir,'objectiveEstData_not_removed');
% InitData_dir=append(dir, idName, '_InitData.mat');
% save(InitData_dir,'InitData');
%
% figure(3);
% C_nom=tf([Kd_nominal+Tf*Kp_nominal,Kp_nominal+Tf*Ki_nominal,Ki_nominal], [Tf, 1, 0]);
% CL_nom=feedback(C_nom*G, 1);
% step(CL_nom)
% hold on
% Kp_BO=results.XAtMinEstimatedObjective.Kp;
% Ki_BO=results.XAtMinEstimatedObjective.Ki;
% Kd_BO=results.XAtMinEstimatedObjective.Kd;
% C_BO=tf([Kd_BO+Tf*Kp_BO,Kp_BO+Tf*Ki_BO,Ki_BO], [Tf, 1, 0]);
% CL_BO=feedback(C_BO*G, 1);
% step(CL_BO, 'r')
% legend('nominal','BO')
% figName=append(dir, idName, '_step_response.png');
% saveas(gcf,figName)
%
% pause;
% close all;
% % FinalBestResult = bestPoint(results)
end

function [objective] = myObjfun_ApproxLoop_perturbed(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat)
global N
global idx
global data
if isempty(N)
    N=1;
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif N==1
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif N==2
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif N==3
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif N==4
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif idx==N_real_repeat
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    G2 = tfest(data,np2);
    
    %     G2_2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.8e0, 0].*(rand(1,1)-0.5));
    %     G2_3=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
    %     G2_4=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
    %     G2_5=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
    %     G2_6=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
    
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= idx +1;
elseif idx==N_real_repeat+1
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
    
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= idx +1;
    
elseif idx==N_real_repeat+2
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
    
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= idx +1;
    
elseif idx==N_real_repeat+3
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
    
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= idx +1;
    
elseif idx==N_real_repeat+4
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
    
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
    
else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
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
end


function [objective] = myObjfun_ApproxLoop(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat)
global N
global idx
global data
if isempty(N)
    N=1;
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
elseif idx==N_real_repeat
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);
    G2 = tfest(data,np2);
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
    idx= 0;
else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G, 1);
    
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;
    %     if abs(stepinfo(CL).Overshoot)<1
    %         objective = abs(1*stepinfo(CL).SettlingTime);
    %     else
    %         objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
    %     end
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

end

function [objective] = myObjfun_Loop(vars, G, Tf)

%     todo move some lines outside with handler@: faster?
C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;
if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end
if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end
w1=1;
w2=500;
objective=ov/w1+st/w2;

% if abs(stepinfo(CL).Overshoot)<1
%     objective = abs(1*stepinfo(CL).SettlingTime);
% else
%     objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
% end
% if isnan(objective)
%     objective=1e10;
% end

end