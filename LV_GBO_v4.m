% GBO version 4
%% ADD PATHS
rehash 
addpath("C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\tfest.m")
addpath("C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\")
addpath C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\@iddata\iddata.m
addpath("C:\mahdi\data_driven_controller\functions")
addpath("C:\mahdi\data_driven_controller\gpml")
tmp_name="exper_72";
tmp_dir=append("C:\mahdi\data_driven_controller\Data\",tmp_name);
dir0=append(tmp_dir,"\N0_Data_",string(expr),"\");
dir=append(tmp_dir,'\GBO_', string(expr), '\');
if not(isfolder(dir))
    mkdir(dir)
end
%% load gain limits
dir_gains=append('C:\mahdi\data_driven_controller\Data\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
load(dir_gains)
%% set params
stat_value=60;
N0=1; %for N0>1 modify
N_iter=50; %number of BO iteration on real plant
% sampleTf=3.1;%check!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
sampleTs=10e-3; % 10ms
step_low=80;
step_high=120;
step_time=5;
nr_repeats=2;
control_mode=1;
Input_mode=2;
gain_angle=0;
Tn_Angle=0;
%% We define the function we would like to optimize
fun = @(perf_Data) ObjFun(perf_Data); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
%% Setup the Gaussian Process (GP) Library
startup;
% Setting parameters for Bayesian Global Optimization
opt.hyp = -1; % Set hyperparameters using MLE.
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
opt.grid_size = 20000;
opt.lt_const = 0.0;
opt.do_cbo = 0; % Do CBO -- use the constraint output from F as well.
opt.save_trace = 0;
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;
% priors
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
% liklihood
likfunc={@likGauss};
% inference method
infer=@infExact;
%% to initialize first the response
if counter==0  %global initialize counter from 0
    Kp=gains0(1);
    Ki=gains0(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
    Input_mode=2;
    start_switch=1;
    % Draw initial candidate grid from a Sobol sequence
    sobol = sobolset(opt.dims);
    hyper_grid = sobol(1:opt.grid_size,:);
    idx_G2=[0];
    counter=1;
    return
end
%%
% load initial dataset
if counter==1
    idx_G2=[0];
    load(append(dir0, 'botrace0.mat'));
    load(append(dir0, 'G2data_init.mat'));
    G2data=G2data_init;
    save(append(dir, 'G2data.mat'),'G2data')
    if LVswitch==0 %for code consistency
        load(append(dir0, 'perf_Data.mat'));
    end
    opt.resume_trace_data = botrace0;
    clear botrace0
elseif counter>1
    load(append(dir, 'trace_file'))
    load(append(dir, 'G2data.mat'))
    load(append(dir, 'LVgains'))
    opt.resume_trace_data = Trace;
    clear Trace
end
addpath("C:\mahdi\data_driven_controller")
if LVswitch==1 % means new exp_Data and perf_Data arrived from real system
    sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
    ytmp = exp_Data(sample_idx,4);
    utmp= exp_Data(sample_idx,5);
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    counter_real= counter_real +1; % counter_real counts number of real system measurements
    % add measured data to BO dataset
    J_measured=ObjFun(mean(perf_Data(end-nr_repeats+1:end,:)));
    Trace=opt.resume_trace_data;
    values=Trace.values;
    values(end+1,1)=J_measured;
    Trace.values=values;
    samples=Trace.samples;
    samples = [samples; LVgains];
    Trace.samples = samples;
    save(append(dir, 'G2data_',num2str(counter_real),'_',num2str(expr),'.mat'),'G2data')
    save(append(dir, 'perf_Data_',num2str(counter_real),'_',num2str(expr),'.mat'), 'perf_Data')
    save(append(dir, 'exp_Data_',num2str(counter_real),'_',num2str(expr),'.mat'), 'exp_Data')
    LVswitch=0;
elseif LVswitch==0  % LVswitch==0 means we need to decide to call either real or surrogate to get data
    [ms,mv,Trace, LVgains,hyper_grid,idx_G2, G2, counter_s] = LV_bayesoptGPML_v4(fun,opt,hyper_grid,counter_s, G2data,idx_G2);
    counter=counter+1; %counter: number of BO iteration in total
    while counter_s>0
        save(append(dir, 'G2_',num2str(idx_G2(end)),'_',num2str(expr),'.mat'), 'G2')
        save(append(dir, 'debug_idx_G2_expr_',num2str(idx_G2(end)),'_',num2str(expr),'.mat'),'idx_G2')
        opt.resume_trace_data = Trace;
        [ms,mv,Trace, LVgains,hyper_grid,idx_G2, G2, counter_s] = LV_bayesoptGPML_v4(fun,opt,hyper_grid,counter_s, G2data,idx_G2);
        counter=counter+1; %counter: number of BO iteration in total
    end
    if counter_s==0 %means we call the real system to get perf_Data
        Kp=LVgains(1);
        Ki=LVgains(2);
        gain_vel=Kp;
        Tn_vel=1/Ki;
        LVswitch=1;
    end
%     else
%         save(append(dir, 'G2_',num2str(idx_G2(end)),'_',num2str(expr),'.mat'), 'G2')
%     end
end
save(append(dir, 'trace_file.mat'),'Trace')
save(append(dir, 'LVgains.mat'),'LVgains')
%% setup automation for the next experiment
if counter_real==N_iter
    LVswitch=0;
    counter_s=0;
    counter_real=0;
    if ~isempty(idx_G2(2:end))
        idx_G2=idx_G2(2:end);
        Trace_removed=opt.resume_trace_data;
        Trace_removed.samples(idx_G2,:)=[];
        Trace_removed.values(idx_G2)=[];
        Trace_removed.post_mus(idx_G2)=[];
        Trace_removed.post_sigma2s(idx_G2)=[];
        Trace_removed.times(idx_G2)=[];
        save(append(dir, 'idx_G2_expr_',num2str(expr),'.mat'),'idx_G2')
    end
    save(append(dir, 'trace_file_expr_',num2str(expr),'.mat'),'Trace_removed')
    expr=expr+1;
    
    Kp=0.5;
    Ki=1.47;
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
    Input_mode=2;
    start_switch=1;
    % Draw initial candidate grid from a Sobol sequence
    sobol = sobolset(opt.dims);
    hyper_grid = sobol(1:opt.grid_size,:);
    idx_G2=[0];
    counter=1;
end
return
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mkdir("C:\mahdi\data_driven_controller\Data\TEST000000000000000000000000\")
% save(append(dir,'hyper_grid_',num2str(counter),'.mat'),'hyper_grid')
% save(append(dir,'counter_',num2str(counter),'.mat'),'counter')
% save(append(dir,'LVgains_',num2str(counter),'.mat'),'LVgains')
% save(append(dir,'counter_s_',num2str(counter),'.mat'),'counter_s')
% save(append(dir,'gains0_',num2str(counter),'.mat'),'gains0')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%