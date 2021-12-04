function GBO
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_GBO_0_3';
sys='DC_motor';
N0=50;
N_iter=200;
repeat_experiment=1;
withSurrogate=false;
N_real_repeat=5;
Nsample=50;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end

%%
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

% % robot-arm System ("Simultaneous computation of model order..., Badaruddin Muhammad et al.")
% num = [-0.0118, 0.0257, 0, 0, 0];
% den = [1, -3.1016, 4.3638, -3.1528, 1.0899, -0.0743];
% ts=1;
% G = d2c(tf(num,den, ts));

% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

%%
% % uncomment to estimate stable gain bounds
% % auto tune
% C_tuned = pidtune(G,'PI');
% % Kd_nominal=C_tuned.Kd;
% Kp_nominal=C_tuned.Kp;
% Ki_nominal=C_tuned.Ki;
%
% % Tf=1e-8;
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
% max_overshoot=0;
% d=1e-1;
% while max_overshoot<50 && ~isnan(max_overshoot)
%     lb=[Kp_nominal-d, Ki_nominal-d];
%     ub=[Kp_nominal+d, Ki_nominal+d];
%     funPS_handle = @(x)funPS(x, G);
%     x = particleswarm(funPS_handle,3,lb,ub);
%
%     Ctmp=tf([x(1), x(1)*x(2)], [1, 0]);
%     CLtmp=feedback(Ctmp*G, 1);
%     d
%     max_overshoot=stepinfo(CLtmp).Overshoot
%     d = d*1.1;
% end
%     function [objective] = funPS(x, G)
%         %     todo move some lines outside with handler@: faster?
%         C=tf([x(1), x(1)*x(2)], [1, 0]);
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
% save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds.mat','Kp_min','Ki_min', 'Kp_max','Ki_max')
if sys=="ball_screw"
    dir_gains=append(tmp_dir,'/', 'ball_screw_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="robot_arm"
    dir_gains=append(tmp_dir,'/', 'robot_arm_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'KpKi_bounds.mat');
end
load(dir_gains)
%%
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

%%
tmp=[];
% % initial values for GP of BO
RAND=rand(N0,1);

% load(append(dir,'RAND.mat'))

Kp = (Kp_max-Kp_min).*RAND + Kp_min;
Ki = (Ki_max-Ki_min).*RAND + Ki_min;
InitobjectiveData = zeros(N0,1);
% todo pay attention how you choose sampleTf?
sampleTf=1.5;
sampleTs=sampleTf/(Nsample-1);
global data
for i=1:N0
    C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
    CL=feedback(C*G, 1);
    InitobjectiveData(i) = myObjfun_Loop(Kp(i), Ki(i), G);
    while isnan(InitobjectiveData(i)) || InitobjectiveData(i)>1000
        RAND(i)=rand(1,1);
        Kp(i) = (Kp_max-Kp_min).*RAND(i) + Kp_min;
        Ki(i) = (Ki_max-Ki_min).*RAND(i) + Ki_min;
        C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
        CL=feedback(C*G, 1);
        InitobjectiveData(i) = myObjfun_Loop(Kp(i), Ki(i), G);
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

%%
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
InitData=table(Kp, Ki);
% 
% %%
% Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
% Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');
% 
% vars=[Kp, Ki];
% if withSurrogate==true
%     if withPerturbed==true
%         fun = @(vars)myObjfun_ApproxLoop_perturbed(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat);
%     else
%         fun = @(vars)myObjfun_ApproxLoop(vars, G, G2, sampleTf, sampleTs, np2, N_real_repeat);
%     end
% else
%     fun = @(vars)myObjfun_Loop(vars, G);
% end
botrace.samples=[Kp, Ki];
botrace.values=InitobjectiveData;
% todo need to correct time?
botrace.times=RAND';
save(append(dir,'trace_file.mat'),'botrace')

%% Initializing the Gaussian Process (GP) Library
addpath ./gpml/
startup;

% Setting parameters for Bayesian Global Optimization
opt = defaultopt(); % Get some default values for non problem-specific options.
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter. 
opt.max_iters = N_iter; % Override the default max_iters value -- probably don't need 100 for this simple demo function.
opt.grid_size = 20000;
%opt.parallel_jobs = 3; % Run 3 jobs in parallel using the approach in (Snoek et al., 2012). Increases overhead of BO, so probably not needed for this simple function.
opt.lt_const = 0.0;
%opt.optimize_ei = 1; % Uncomment this to optimize EI/EIC at each candidate rather than optimize over a discrete grid. This will be slow.
%opt.grid_size = 300; % If you use the optimize_ei option
opt.do_cbo = 0; % Do CBO -- use the constraint output from F as well.
opt.save_trace = 1;
%opt.trace_file = 'demo_trace.mat';
%matlabpool 3; % Uncomment to do certain things in parallel. Suggested if optimize_ei is turned on. If parallel_jobs is > 1, bayesopt does this for you.
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;
%% We define the function we would like to optimize
fun = @(X) myObjfun_Loop(X(1), X(2), G); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.

% Let's plot grid of points just to see what we are trying to optimize
clf;
Kp_range=Kp_max-Kp_min;
resol=10;
Kp_surf_resol=Kp_range/resol;
Ki_range=Ki_max-Ki_min;
Ki_surf_resol=Ki_range/resol;
[kp_pt,ki_pt]=meshgrid(Kp_min:Kp_surf_resol:Kp_max,Ki_min:Ki_surf_resol:Ki_max);
j_pt=zeros(size(kp_pt));
c_pt=zeros(size(kp_pt));
for i=1:size(kp_pt,1)
    for j=1:size(kp_pt,2)
        [l,c]=myObjfun_Loop(kp_pt(i,j),ki_pt(i,j),G);
        j_pt(i,j)=l;
        c_pt(i,j)=c;
    end
end
% [j_pt,c_pt]=myObjfun_Loop(kp_pt(:),ki_pt(:),G);
j_pt(c_pt>opt.lt_const)=NaN;
surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
xlabel('Kp')
ylabel('Ki')
zlabel('J')
drawnow;

%% Start the optimization
fprintf('Optimizing hyperparamters of function "samplef.m" ...\n');
[ms,mv,T] = bayesoptGPML(fun,opt);   % ms - Best parameter setting found
                               % mv - best function value for that setting L(ms)
                               % T  - Trace of all settings tried, their function values, and constraint values.
                              
%% Print results
fprintf('******************************************************\n');
fprintf('Best hyperparameters:      P1=%2.4f, P2=%2.4f\n',ms(1),ms(2));
fprintf('Associated function value: F([P1,P2])=%2.4f\n',mv);
fprintf('******************************************************\n');

%% Draw optimium
hold on;
plot3([ms(1) ms(1)],[ms(2) ms(2)],[max(j_pt(:)) min(j_pt(:))],'r-','LineWidth',2);

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif idx==N_real_repeat
    N = N+1;

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
    w1=1;     w2=500;     objective=ov/w1+st/w2;

    idx= idx +1;
elseif idx==N_real_repeat+1
    N = N+1;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= idx +1;

elseif idx==N_real_repeat+2
    N = N+1;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= idx +1;

elseif idx==N_real_repeat+3
    N = N+1;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

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

    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
end


function [objective] = myObjfun_ApproxLoop(vars, G, G2, sampleTf, sampleTs, np2, N_real_repeat)
global N
global idx
global data
if isempty(N)
    N=1;
    objective=myObjfun_Loop(vars, G2);
    idx= 0;
elseif idx==N_real_repeat
    N = N+1;
    G2 = tfest(data,np2);
    objective=myObjfun_Loop(vars, G2);
    idx= 0;
else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    objective=myObjfun_Loop(vars, G);
    C=tf([vars.Kp,vars.Kp*vars.Ki], [1, 0]);
    CL=feedback(C*G, 1);
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end

end

function [objective, constraints] = myObjfun_Loop(Kp, Ki, G)

%     todo move some lines outside with handler@: faster?
C=tf([Kp,Kp*Ki], [1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,1.0]).RiseTime;
ITAE = trapz(t, t.*abs(e));

if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end

if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=1e5;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=1e5;
end

w1=0.1;
w2=1;
w3=1;
w4=0.5;
objective=ov/w1+st/w2+Tr/w3+ITAE/w4;
constraints=-1;
end