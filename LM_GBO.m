% Linear Motor
% GBO version 4
%% clean, set directories, start OPC server
clear all; clc; close all;
addpath ./gpml/
addpath("/home/mahdi/ETHZ/HaW/linear_motor")
startup;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
idName= 'LM_test';
sys='DC_motor';
dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end
% Connect to OPC server
connect(uaObj,'tm','sX7Cswc34wjvwq'); % SPS Loggin
%% set hyperparameters
objective_noise=true;
% set seed of all random generations
rng(1,'twister');
N0=1; %number of initial data
N_expr=2;
N_iter=50;
N_iter=N_iter+N0;
Nsample=150;
sampleTf=2.5; %based on the min and max settling time equal to 1.3 and 19 seconds inside the feasible set "KpKi_bounds_new_2.mat" we choose 1.5 for DC motor plant with speed sensor pole 9.918e-5
sampleTs=sampleTf/(Nsample-1);
sampleTinit=0.0;
lt_const=0.0;
initRant="latin"; %build initial set randomnly witith latin hypercubes
npG2=2;
%% load gain limits (feasible set)
if sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'LM_KpKd_bounds.mat');
end
load(dir_gains)
%% build initial dataset (N0)
if initRant=="latin"
    % latin hypercube samples
    % sample from latin (denoted as ltn) hypercube
    RAND_all_expr=zeros(N0,N_expr);
    for expr=1:N_expr
        RAND = sort(lhsdesign(N0,1));
        RAND_all_expr(:,expr)=RAND;
    end
    save(append(dir,'RAND_ltn_all.mat'),'RAND_all_expr')
end
%% Setup the Gaussian Process (GP) Library
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
opt.grid_size = 20000;
opt.lt_const = lt_const;
opt.do_cbo = 0; % Do CBO -- use the constraint output from F as well.
opt.save_trace = 0;
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;
%% We define the function we would like to optimize
fun = @(X, surrogate)ObjFun_Guided_v4(X, surrogate, sampleTs, npG2);
%% Start the optimization
global N
global idx
global G2data
global N_G2
global y_s
global idx_G2
step_high=40;
% each experiment is the entire iterations starting with certain initial set
for expr=1:1:N_expr
    expr_G2rmse=[];
    y_s=[];
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
    Ki_ltn = (Ki_max-Ki_min).*RAND + Ki_min;
    J_ltn = zeros(N0,1);
    for i=1:N0
        exp_Data=LinMotor(Kp_ltn(i), Kd_ltn(i));
        J_ltn(i) = ObjFun(exp_Data,[],[]);
        sample_idx=exp_Data.r(:,3)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        ytmp = exp_Data.actPos((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
        utmp = exp_Data.actCur((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
        if i==1
            G2data = iddata(ytmp,utmp,sampleTs);
        else
            G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
        end
        %     get data for sigma_surrogate estimation
        npG2=2;
        nzG2=1;
        Options = tfestOptions('Display','off');
        Options.InitialCondition = 'backcast';
        Options.EnforceStability=1;
        G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
        surrogate_objective=ObjFun([Kp_ltn(i), Ki_ltn(i)], G2, [Kp,Kd]);
        y_s=[y_s;surrogate_objective];
        load(append(dir, 'botrace0.mat'))
        opt.resume_trace_data = botrace0;
        clear botrace0
    end
    idx_G2=[];
    % todo check concept of max_iters?
    opt.max_iters = size(opt.resume_trace_data.samples,1)+N_iter-1;
    [ms,mv,Trace_tmp] = LM_bayesoptGPML_v4(fun,opt,N0,y_s);
    G2_samples=Trace_tmp.samples(idx_G2,:);
    G2_values=Trace_tmp.values(idx_G2);
    G2_post_mus=Trace_tmp.post_mus(idx_G2);
    G2_post_sigma2s=Trace_tmp.post_sigma2s(idx_G2);
    % keep surrogate model data seperately for plots
    Trace_tmp.G2_samples=G2_samples;
    Trace_tmp.G2_values=G2_values;
    Trace_tmp.G2_post_mus=G2_post_mus;
    Trace_tmp.G2_post_sigma2s=G2_post_sigma2s;
    % remove previos data of older surrogate(G2) model
    Trace_tmp.samples(idx_G2,:)=[];
    Trace_tmp.values(idx_G2)=[];
    Trace_tmp.post_mus(idx_G2)=[];
    Trace_tmp.post_sigma2s(idx_G2)=[];
    Trace_tmp.times(idx_G2)=[];
    Trace_tmp.hyp_GP_lik(idx_G2)=[];
    Trace_tmp.hyp_GP_cov(idx_G2,:)=[];
    Trace_tmp.hyp_GP_mean(idx_G2,:)=[];
    Trace_tmp.y_s=y_s;
    Trace(expr)=Trace_tmp;
    clearvars Trace_tmp
    save(append(dir, 'trace_file.mat'),'Trace')
    save(append(dir, 'idx_G2.mat'),'idx_G2')
    save(append(dir, 'G2rmse_', num2str(expr),'.mat'),'expr_G2rmse')
end
disconnect_OPCUA(uaObj)
%%
function [objective] = ObjFun(exp_data, G2, gains)
if isempty(G2)==1
    sample_idx=exp_data.r(:,1)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    ytmp = exp_data.actPos((tmp_idx(1)-10):tmp_idx(end))-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-10):tmp_idx(end))-u_offset;
    if exist('G2data')
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    else
        G2data = iddata(ytmp,utmp,sampleTs);
    end
    reference0=0;
    reference=10;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_high_all=[y_high_all,y_high];
    t_high_all=[t_high_all,t_high];
    y_init=mean(exp_data.actPos_all((tmp_idx(1)-60):(tmp_idx(1)-10),exper))-y_offset;
    y_final=mean(exp_data.actPos_all((tmp_idx(end)-60):(tmp_idx(end)-10),exper))-y_offset;
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));
    e_ss=abs(y_final-reference);
elseif isempty(G2)==0 %when we use surrogate to estimate objective
    P=gains(1);
    D=gains(2);
    F=0.001;
    Kp = P;
    Ti = inf;
    Td = D/P;
    N=D/(P*F);
    Ts = 0.001;
    C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
    CL=feedback(C*G2, 1);
    reference0=0;
    reference=10;
    t_high=(11*Ts):Ts:(3.01-Ts);
    t_low=0:Ts:(10*Ts);
    step_high=reference.*ones(length(t_high),1);
    step_low=reference0.*ones(length(t_low),1);
    t=[t_low,t_high]';
    r=[step_low;step_high];
    y2=lsim(CL,r,t);
    y_high=y2(t>(.01)); %TODO check pay attention
    t_high=t(t>(.01));%TODO check    
    y_init=mean(exp_data.actPos_all((tmp_idx(1)-60):(tmp_idx(1)-10),exper));
    y_final=mean(exp_data.actPos_all((tmp_idx(end)-60):(tmp_idx(end)-10),exper));
    e_ss=abs(y_final-reference);
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));
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
function [objective, N_G2, idx_G2, y_s] = ObjFun_Guided_v4(X, surrogate, sampleTs, npG2)
global N
global G2data
global N_G2
global idx_G2
global y_s

N=N+1;
if surrogate==true
    npG2=2;
    nzG2=1;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
    objective=ObjFun([], G2, [Kp,Kd]);
    N_G2=N_G2+1;
    idx_G2= [idx_G2;N];
elseif surrogate==false
    exp_Data=LinMotor(X(1), X(2));
    objective = ObjFun(exp_Data,[],[]);

    sample_idx=exp_Data.r(:,3)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    ytmp = exp_Data.actPos((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
    utmp = exp_Data.actCur((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));

    %     get data for sigma_surrogate estimation
    npG2=2;
    nzG2=1;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
    
    surrogate_objective=ObjFun(X, G2, [Kp,Kd]);
    y_s=[y_s;surrogate_objective];
end
fprintf('N= %d \n', N);
fprintf('N_G2= %d \n', N_G2);
end

function nh = num_hypers(func,opt)
str = func(1);
nm = str2num(str);
if ~isempty(nm)
    nh = nm;
else
    if isequal(str, 'D*1')
        nh = opt.dims * 1;
    elseif isequal(str,'(D+1)')
        nh = opt.dims + 1;
    elseif isequal(str,'(D+2)')
        nh = opt.dims + 2;
    elseif isequal(str,'D')
        nh = opt.dims ;
    else
        error('bayesopt:unkhyp','Unknown number of hyperparameters asked for by one of the functions');
    end
end
end
%%
function exp_data=LinMotor(Kp,Kd)
% %% Load Trajectory
% % load("dem_x/dem_x_9_mm.mat")
% t = (0.001:0.001:7);
% r= 10.*(t>2)+30-10.*(t>5);
% % % set reference input
% write_OPCUA(uaObj,'arrDemPos', r);
% write_OPCUA(uaObj,'arrShowPos', r);
% pause(1);
% % Set P and D gain
% write_OPCUA(uaObj,'LQR_P_x', Kp);
% write_OPCUA(uaObj,'LQR_D_v', Kd);
% pause(0.2);
% % perform experiment
% write_OPCUA(uaObj,'Go', 1); % Perform Experiment
% pause(7.5);
% % read the results
% actPos = read_OPCUA(uaObj,'arrActPos')';
% actVel = read_OPCUA(uaObj,'arrActVel')';
% actCur = read_OPCUA(uaObj,'arrActCur')';
% exp_data.actPos=actPos;
% exp_data.actVel=actVel;
% exp_data.actCur=actCur;
% exp_data.r=r';
% exp_data.t=t';

load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_test.mat")
end