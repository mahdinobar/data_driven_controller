% Linear Motor (offline dataset)
% GBO version 5(remove surrogate from BO Dataset 
%% clean, set directories, start OPC server
clear all; clc; close all;
addpath ./gpml/
addpath("/home/mahdi/ETHZ/HaW/linear_motor")
startup;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data';
idName= 'LM_v5_101';
sys='LM';
isGBO=true;
if isGBO==true
    dir=append(tmp_dir,'/', idName, '/GBO/');
else
    dir=append(tmp_dir,'/', idName, '/BO/');
end
if not(isfolder(dir))
    mkdir(dir)
end

%% set hyperparameters
% set seed of all random generations
rng(1,'twister');
N0=1; %number of initial data
N_expr=50;
N_iter=30;
N_iter=N_iter+N0;
sampleTs=0.001;
sampleTinit=0.0;
lt_const=0.0;
initRant="latin"; %build initial set randomnly witith latin hypercubes
npG2=2;
%% load gain limits (feasible set)
if sys=="LM"
    dir_gains='/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_KpKd_bounds.mat';
end
load(dir_gains)
%% load and prepare LM offline dataset
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data.mat")
global P_crop_safe D_crop_safe exp_data_crop_safe
idx_crop_safe=logical((P_safe<Kp_max).*(P_safe>Kp_min).*(D_safe<Kd_max).*(D_safe>Kd_min));
P_crop_safe=P_safe(idx_crop_safe);
D_crop_safe=D_safe(idx_crop_safe);

exp_data_crop_safe=exp_data_safe;
exp_data_crop_safe.actPos_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actCur_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actVel_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.P(~idx_crop_safe)=[];
exp_data_crop_safe.D(~idx_crop_safe)=[];
%% build initial dataset (N0)
if initRant=="latin"
    if isGBO==true
        load(append(tmp_dir,'/', idName, '/RAND_ltn_all.mat'))
    else
    % latin hypercube samples
    % sample from latin (denoted as ltn) hypercube
    RAND_all_expr=zeros(N0,N_expr);
    for expr=1:N_expr
        RAND = sort(lhsdesign(N0,1));
        RAND_all_expr(:,expr)=RAND;
    end
    save(append(tmp_dir,'/', idName, '/RAND_ltn_all.mat'),'RAND_all_expr')
    end
end
%% Setup the Gaussian Process (GP) Library
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Kd_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Kd_max]; % Vector of maximum values for each parameter.
% opt.grid_size = 20000;
opt.grid=[P_crop_safe,D_crop_safe]; %use grid for offline LM dataset
opt.lt_const = lt_const;
opt.do_cbo = 0; % Do CBO -- use the constraint output from F as well.
opt.save_trace = 0;
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;
%% We define the function we would like to optimize
fun = @(X, surrogate)ObjFun_Guided_v5(X, surrogate);
%% Start the optimization
global N
global idx
global G2data
global N_G2
step_high=40;
% each experiment is the entire iterations starting with certain initial set
for expr=1:1:N_expr
    fprintf('>>>>>experiment: %d \n', expr);
    N=0;
    idx=[];
    N_G2=0;
    G2_samples=[];
    G2_values=[];
    G2_post_mus=[];
    G2_post_sigma2s=[];
    % create initial dataset per experiment
    RAND=RAND_all_expr(:,expr);
    range_kp=Kp_max-Kp_min;
    range_kd=Kd_max-Kd_min;
    Kp_ltn = (Kp_max-Kp_min).*RAND + Kp_min;
    Kd_ltn = (Kd_max-Kd_min).*RAND + Kd_min;

    [~,I_tmp]=min((P_crop_safe-Kp_ltn).^2+(D_crop_safe-Kd_ltn).^2);
    Kp_ltn=P_crop_safe(I_tmp);
    Kd_ltn=D_crop_safe(I_tmp);
    J_ltn = zeros(N0,1);
    for i=1:N0
        exp_data=LinMotor(Kp_ltn(i), Kd_ltn(i));
        J_ltn(i) = ObjFun(exp_data,[],[]);
        sample_idx=exp_data.r==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_offset=exp_data.actPos(tmp_idx(1)-10);
        u_offset=exp_data.actCur(tmp_idx(1)-10);
        % use 50 ms of data after step high for G2
        ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
        utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
        if i==1
            G2data = iddata(ytmp,utmp,sampleTs);
        else
            G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
        end
    end
    %%
    % set initial dataset
    X_ltn=[Kp_ltn, Kd_ltn];
    y_ltn=J_ltn;
    botrace0.samples=X_ltn;
    botrace0.values=y_ltn;
    botrace0.idx_G2_samples=[];
    % todo need to correct time?
    botrace0.times=RAND';
    opt.resume_trace_data = botrace0;
    clear botrace0
    % todo check concept of max_iters?
    opt.max_iters = size(opt.resume_trace_data.samples,1)+N_iter-1;
    [ms,mv,Trace_tmp] = LM_bayesoptGPML_v5(fun,opt,N0);
    Trace(expr)=Trace_tmp;
    clearvars Trace_tmp
    save(append(dir, 'trace_file.mat'),'Trace')
end

%%
function [objective] = ObjFun(exp_data, G2, gains)
step_high=40;
sampleTs=0.001;
if isempty(G2)==1
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    if exist('G2data')
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    else
        G2data = iddata(ytmp,utmp,sampleTs);
    end
    reference0=0;
    reference=10;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=mean(exp_data.actPos((tmp_idx(1)-60):(tmp_idx(1)-10)))-y_offset;
    y_final=mean(exp_data.actPos((tmp_idx(end)-5):(tmp_idx(end))))-y_offset;
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(3*Tr*1000)), t_high(1:ceil(3*Tr*1000))'.*abs(e(1:ceil(3*Tr*1000))));
    e_ss=abs(y_final-reference);
elseif isempty(G2)==0 %when we use surrogate to estimate objective
    P=gains(1);
    D=gains(2);
    F=0.001;
    Kp = P;
    Ti = inf;
    Td = D/P;
    N=D/(P*F);
    Ts = sampleTs;
    C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
    CL=feedback(C*G2, 1);
    reference0=0;
    reference=10;
    t_high=(11*Ts):Ts:(0.060-Ts);
    t_down=0:Ts:(10*Ts);
    step_high=reference.*ones(length(t_high),1);
    step_down=reference0.*ones(length(t_down),1);
    t=[t_down,t_high]';
    r=[step_down;step_high];
    y2=lsim(CL,r,t);
    y_high=y2(t>(.01)); %TODO check pay attention
    t_high=t(t>(.01));%TODO check    
    y_init=0;
    y_final=mean(y_high(end-5:end));
    e_ss=abs(y_final-reference);
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(3*Tr*1000))', t_high(1:ceil(3*Tr*1000)).*abs(e(1:ceil(3*Tr*1000))));
    e_ss=abs(y_final-reference);
end
if isnan(ov) || isinf(ov) || ov>1
    ov=1;
end
if isnan(st) || isinf(st) || st>3
    st=3;
end
if isnan(Tr) || isinf(Tr) || Tr>3
    Tr=3;
end
if isnan(ITAE) || isinf(ITAE) || ITAE>30
    ITAE=30;
end
if isnan(e_ss) || isinf(e_ss) || e_ss>10
    e_ss=10;
end
w_mean_grid=[0.1506, 0.0178, 0.0940, 0.0190, 0.4968]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1.2, 1.05, 0.98, 1, 1.1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)+e_ss*w(5);
end
%%
function [objective, N_G2] = ObjFun_Guided_v5(X, surrogate)
global N
global G2data
global N_G2

N=N+1;
if surrogate==true
    npG2=2;
    nzG2=1;
    sampleTs=0.001;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', sampleTs);
    objective=ObjFun([], G2, X);
    N_G2=N_G2+1;
elseif surrogate==false
    exp_data=LinMotor(X(1), X(2));
    objective = ObjFun(exp_data,[],[]);
    step_high=40;
    sampleTs=0.001;
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
end
fprintf('N= %d \n', N);
fprintf('N_G2= %d \n', N_G2);
end
%%
function exp_data=LinMotor(Kp,Kd)
global exp_data_crop_safe
i=find((exp_data_crop_safe.P==Kp).*(exp_data_crop_safe.D==Kd));
exp_data.actPos=exp_data_crop_safe.actPos_all(:,i);
exp_data.actVel=exp_data_crop_safe.actVel_all(:,i);
exp_data.actCur=exp_data_crop_safe.actCur_all(:,i);
exp_data.r=exp_data_crop_safe.r;
exp_data.t=exp_data_crop_safe.t;
exp_data.P=exp_data_crop_safe.P(i);
exp_data.D=exp_data_crop_safe.D(i);
end