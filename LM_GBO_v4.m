% Linear Motor (offline dataset)
% GBO version 4 (keep only last surrogate)
%% clean, set directories, start OPC server
clear all; clc; close all;
addpath ./gpml/
addpath("/home/mahdi/ETHZ/HaW/linear_motor")
startup;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data';
idName= 'LM_v4_111_debug';
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
N_expr=10;
N_iter=150;
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
fun = @(X, surrogate)ObjFun_Guided_v4(X, surrogate);
%% Start the optimization
global N
global idx
global G2data
global N_G2
step_high=40;
step_down=30;
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
        vtmp = diff(ytmp((50+3):end))./sampleTs;
        utmp = utmp((50+3):(end-1));
        if i==1
            G2data = iddata(vtmp,utmp,sampleTs);
        else
            G2data = merge(G2data, iddata(vtmp,utmp,sampleTs));
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
    [ms,mv,Trace_tmp] = LM_bayesoptGPML_v4(fun,opt,N0);
    Trace(expr)=Trace_tmp;
    clearvars Trace_tmp
    save(append(dir, 'trace_file.mat'),'Trace')
end

%%
function [objective] = ObjFun(exp_data, G2, gains)
sampleTs=0.001;
if isempty(G2)==1
    step_high=40;
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    reference0=0;
    reference=10;
    y_high=ytmp(50:end); %todo check
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=mean(exp_data.actPos((tmp_idx(1)-60):(tmp_idx(1)-10)))-y_offset;
    y_final=mean(exp_data.actPos((tmp_idx(end)-60):(tmp_idx(end)-10)))-y_offset;
    % manually calculate settling time for server because server lsiminfo is wrong
    i_st = max(find(abs(y_high-y_final)>0.02*(y_final-y_init)));
    st=t_high(i_st+1);
    if isnan(st)
        st=3;
    end
    if max(y_high)>reference
        ov=max(0,(max(y_high)-y_init)/(y_final-y_init)-1);
    else
        ov=0;
    end
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=y_high-reference;
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), abs(e(1:ceil(5*Tr*1000))));
    e_ss=abs(y_final-reference);
elseif isempty(G2)==0 %when we use surrogate to estimate objective
    F=0.001;
    P=gains(1)/512;
    D=gains(2)/768;
    I=0;
    reference0=0;
    reference=10;
    G2c=d2c(G2);
    G2_num=G2c.Numerator{1};
    G2_den=G2c.Denominator{1};

    mdlWks = get_param('DT','ModelWorkspace');
    assignin(mdlWks,'sampleTs',sampleTs)
    assignin(mdlWks,'P',P)
    assignin(mdlWks,'D',D)
    assignin(mdlWks,'I',I)
    assignin(mdlWks,'F',F)
    assignin(mdlWks,'reference0',reference0)
    assignin(mdlWks,'reference',reference)
    assignin(mdlWks,'G2_den',G2_den)
    assignin(mdlWks,'G2_num',G2_num)
    simOut = sim("DT.slx");

    y2=simOut.yout{1}.Values.Data(1:10:end-1);
    t=simOut.tout(1:10:end-1);
    y_high=y2(t>(50*sampleTs)); %TODO check pay attention
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=0;
    y_final=mean(y_high(end-5:end));
    % manually calculate settling time for server because server lsiminfo is wrong
    i_st = max(find(abs(y_high-y_final)>0.02*(y_final-y_init)));
    st=t_high(i_st+1);
    if isnan(st)
        st=3;
    end
    if max(y_high)>reference
        ov=max(0,(max(y_high)-y_init)/(y_final-y_init)-1);
    else
        ov=0;
    end
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=y_high-reference;
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), abs(e(1:ceil(5*Tr*1000))));
    e_ss=abs(y_final-reference);

%     %% debug
%     exp_data=LinMotor(gains(1),gains(2));
%     step_high=40;
%     sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
%     tmp_idx=find(sample_idx>0);
%     tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
%     tmp_idx=tmp_idx(tmp_idx_2);
%     y_offset=exp_data.actPos(tmp_idx(1)-10);
%     u_offset=exp_data.actCur(tmp_idx(1)-10);
%     % use 50 ms of data after step high for G2
%     ytmp = exp_data.actPos((tmp_idx(1)-49):tmp_idx(1)+70)-y_offset;
%     utmp = exp_data.actCur((tmp_idx(1)-49):tmp_idx(1)+70)-u_offset;
%     reference0=0;
%     reference=10;
%     close(figure(200))
%     figure(200); 
%     set(gcf, 'Position', get(0, 'Screensize'));
%     hold on; 
%     plot(t,simOut.yout{3}.Values.Data(1:end-1),"k");
%     plot(t,simOut.yout{2}.Values.Data(1:10:end-1),"b");
%     plot(t,y2,"r");
%     plot(t,ytmp,"g");
%     plot(t,utmp,"--g");
%     RMSE=sqrt(sum((ytmp(50:end)-y2(50:end)).^2));
%     title(append("RMSE=",string(RMSE),", P=",string(gains(1)),", D=",string(gains(2))))
%     pause;
end
if isnan(ov) || isinf(ov) || ov>1
    ov=1;
end
if isnan(st) || isinf(st) || st>70e-3
    st=70e-3;
end
if isnan(Tr) || isinf(Tr) || Tr>70e-3
    Tr=70e-3;
end
if isnan(ITAE) || isinf(ITAE) || ITAE>1
    ITAE=1;
end
if isnan(e_ss) || isinf(e_ss) || e_ss>10
    e_ss=10;
end
if Tr==0
    Tr=sampleTs;
end
if ITAE==0
    ITAE=sampleTs*(reference-reference0);
end
if st==0
    st=sampleTs;
end

w_mean_grid=[0.0732, 0.0425, 0.0117, 0.2044, 0.0339];%[0.1506, 0.0178, 0.0940, 0.0190, 0.4968]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1.2, 1.05, 0.98, 1, 1.1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)+e_ss*w(5);
end
%%
function [objective] = ObjFun_Guided_v4(X, surrogate)
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
    G2v = tfest(G2data, npG2,nzG2,Options, 'Ts', sampleTs);
    z = tf('z',sampleTs);
    G2 = G2v * sampleTs/2 * (z+1)/(z-1);
    objective=ObjFun([], G2, X);
    N_G2=N_G2+1;
elseif surrogate==false
    exp_data=LinMotor(X(1), X(2));
    objective = ObjFun(exp_data,[],[]);
    step_high=40;
    step_down=30;
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
    vtmp = diff(ytmp((50+3):end))./sampleTs;
    utmp = utmp((50+3):(end-1));
    G2data = merge(G2data, iddata(vtmp,utmp,sampleTs));
    save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/LM_v4_111_debug/G2data.mat","G2data")
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