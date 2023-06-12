% Linear Motor
% GBO version 4
%% clean start, set directories
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

%% set hyperparameters
isGBO=true;
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
% uncomment for isGBO
npG2=2;

%% load gain limits (feasible set)
if sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'LM_KpKd_bounds.mat');
end
load(dir_gains)

%% build initial dataset (N0)
if initRant=="latin"
    % latin hypercube samples
    if isGBO
        % load same samples used for BO
        load(append(dir,'RAND_ltn_all.mat'), 'RAND_all_expr')
    else
        % sample from latin (denoted as ltn) hypercube
        RAND_all_expr=zeros(N0,N_expr);
        for expr=1:N_expr
            RAND = sort(lhsdesign(N0,1));
            RAND_all_expr(:,expr)=RAND;
        end
        save(append(dir,'RAND_ltn_all.mat'),'RAND_all_expr')
    end
end

%% Setup the Gaussian Process (GP) Library
% Setting parameters for Bayesian Global Optimization
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
opt.grid_size = 20000;
%opt.parallel_jobs = 3; % Run 3 jobs in parallel using the approach in (Snoek et al., 2012). Increases overhead of BO, so probably not needed for this simple function.
opt.lt_const = lt_const;
%opt.optimize_ei = 1; % Uncomment this to optimize EI/EIC at each candidate
%rather than optimize over a discrete grid. This will be slow but requires
%less grid size.
%opt.grid_size = 300; % If you use the optimize_ei option
opt.do_cbo = 0; % Do CBO -- use the constraint output from F as well.
opt.save_trace = 0;
%opt.trace_file = 'demo_trace.mat';
%matlabpool 3; % Uncomment to do certain things in parallel. Suggested if optimize_ei is turned on. If parallel_jobs is > 1, bayesopt does this for you.
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;

%% We define the function we would like to optimize
if isGBO==true
    fun = @(X, surrogate)ObjFun_Guided_v4(X, surrogate, sampleTs, npG2);
else
    fun = @(X) ObjFun(exp_Data, G2, X); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Start the optimization
global N
global idx
global G2data
global N_G2
global y_s
global idx_G2
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
        [r,u,y]=LinMotor(Kp_ltn(i), Kd_ltn(i));
        exp_Data=[r,u,y];
        J_ltn(i) = ObjFun(exp_Data);
        if isGBO==true
            sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
            tmp_idx=find(sample_idx>0);
            tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
            tmp_idx=tmp_idx(tmp_idx_2);
            ytmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
            utmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
            if i==1
                G2data = iddata(ytmp,utmp,sampleTs);
            else
                G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
            end
            %     get data for sigma_surrogate estimation
            G2=tfest(G2data, npG2);
            surrogate_objective=ObjFun([Kp_ltn(i), Ki_ltn(i)], G2, false);
            y_s=[y_s;surrogate_objective];
            load(append(dir, 'botrace0.mat'))
            opt.resume_trace_data = botrace0;
            clear botrace0
        else
            % set initial dataset
            X_ltn=[Kp_ltn, Ki_ltn];
            y_ltn=J_ltn;
            botrace0.samples=X_ltn;
            botrace0.values=y_ltn;
            botrace0.times=RAND';
            opt.resume_trace_data = botrace0;
            save(append(dir, 'botrace0.mat'),'botrace0')
            clear botrace0
        end
    end

    idx_G2=[];
    % todo check concept of max_iters?
    opt.max_iters = size(opt.resume_trace_data.samples,1)+N_iter-1;
    [ms,mv,Trace_tmp] = LM_bayesoptGPML_v4(fun,opt,N0,y_s, isGBO);
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
    %     %         remove irrelevant fields for ease of load
    %     Trace_tmp=rmfield(Trace_tmp,'post_sigma2s_record');
    %     Trace_tmp=rmfield(Trace_tmp,'hyper_grid_record');
    %     Trace_tmp=rmfield(Trace_tmp,'post_mus_record');
    Trace(expr)=Trace_tmp;
    clearvars Trace_tmp
    if isGBO==true
        save(append(dir, 'trace_file.mat'),'Trace')
        save(append(dir, 'idx_G2.mat'),'idx_G2')
        save(append(dir, 'G2rmse_', num2str(expr),'.mat'),'expr_G2rmse')
    else
        save(append(dir, 'trace_file_BO.mat'),'Trace')
    end
end

function [objective, constraints] = ObjFun(exp_Data, G2, X)
if isempty(G2)==1
    sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    ytmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
    %calculate performance data based on experimental step response measurements
    reference0=0;
    reference=40;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    e=abs(y_high-reference);
    ITAE = trapz(t_high, t_high'.*abs(e));
    S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.05);
    st=S.SettlingTime;
    if isnan(st)
        st=5;
    end
    ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
    Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
else
    sample_idx=exp_data.r_all(:,1)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    ytmp = exp_data.actPos_all((tmp_idx(1)-10):tmp_idx(end),exper)-y_offset;
    utmp = exp_data.actCur_all((tmp_idx(1)-10):tmp_idx(end),exper)-u_offset;
    if exist('G2data')
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    else
        G2data = iddata(ytmp,utmp,sampleTs);
    end
    % G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    %calculate performance data based on experimental step response measurements
    reference0=0;
    reference=10;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
    Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));
    perf_Data=[ov,Tr,st,ITAE];


    ov=abs(perf_Data(1,1));
    st=perf_Data(1,3);
    Tr=perf_Data(1,2);
    ITAE = perf_Data(1,4);
    e_ss = perf_Data(1,5);
end

if isnan(ov) || isinf(ov) || ov>1
    ov=1;
end

if isnan(st) || isinf(st) || st>1e5
    st=3;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=3;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=30;
end

w_mean_grid=[0.1506, 0.0178, 0.0940, 0.0079, 0.4968]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1.02, 1.02, 0.98, 1, 1.02];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);

constraints=-1;
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
    G2=tfest(G2data, npG2);
    objective=ObjFun([], G2, X);
    N_G2=N_G2+1;
    idx_G2= [idx_G2;N];
elseif surrogate==false
    [r,u,y]=LinMotor(X(1), X(2));
    exp_Data=[r,u,y];
    objective = ObjFun(exp_Data,[],[]);
    sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    ytmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
    utmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    %     get data for sigma_surrogate estimation
    G2=tfest(G2data, npG2);
    surrogate_objective=ObjFun(X, G2, false);
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
function [r,u,y]=LinMotor(Kp,Kd)
% % Connect to OPCUA
% uaObj = connect_OPCUA('192.168.188.21');
% Load Trajectory
% load("dem_x/dem_x_9_mm.mat")
t = (0.001:0.001:7);
r= t>3.5;

% % set reference input
% % write_OPCUA(uaObj,'arrDemPos', dem_x*1000);
% write_OPCUA(uaObj,'arrDemPos', r);
% pause(1);

% Run with PID and DOB (P=2584 I=0 D=0.015 N=740)
write_OPCUA(uaObj,'Go', 1); % Perform Experiment
pause(7.5);
% Read
close all
actPos = read_OPCUA(uaObj,'arrActPos')/1000;
actVel = read_OPCUA(uaObj,'arrActVel');
actCur = read_OPCUA(uaObj,'arrActCur');
actNomCur = read_OPCUA(uaObj,'arrActNomCur');

actPos = actPos(350:end)';
actPos = actPos - actPos(1);
actVel = actVel(350:end)';
actCur = actCur(350:end)';
actNomCur = actNomCur(350:end)';

figure
plot(actPos);
xlabel('Time [s]'); ylabel('Pos [m]')

figure
plot(actVel)
xlabel('Time [s]'); ylabel('Vel [m/s]')

figure
hold on
plot(actCur)
plot(actNomCur)
xlabel('Time [s]'); ylabel('Current [A]')
legend('actCur','NomCur')

% time = linspace(0,length(actPos)/1000,length(actPos))';

ExprData = timetable(actNomCur,actCur,actVel,actPos,'SampleRate',1000);
save('ExprData\Expr_9.mat','ExprData')
%
% Disconnect from OPCUA
disconnect_OPCUA(uaObj)
end