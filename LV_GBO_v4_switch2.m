% GBO version 4_switch2 switching sigma_GP/sigma_s
%% ADD PATHS
rehash 
addpath("C:\Program Files\MATLAB\R2022b\toolbox\ident\ident\tfest.m")
addpath("C:\Program Files\MATLAB\R2022b\toolbox\ident\ident\")
addpath C:\Program Files\MATLAB\R2022b\toolbox\ident\ident\@iddata\iddata.m
addpath("C:\mahdi\data_driven_controller\functions")
addpath("C:\mahdi\data_driven_controller\gpml")
tmp_name="exper_72_6";
tmp_dir=append("C:\mahdi\data_driven_controller\Data\",tmp_name);
dir0=append(tmp_dir,"\N0_Data_",string(expr),"\");
dir=append(tmp_dir,'\GBO_sw2_v4_', string(expr), '\');
if not(isfolder(dir))
    mkdir(dir)
end
%% load gain limits
dir_gains=append('C:\mahdi\data_driven_controller\Data\DC_motor_gain_bounds\KpKi_bounds_new_3.mat');
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
    counter=1;
    return
end
%%
% load initial dataset
load(append(dir0, 'y_offset.mat'));
load(append(dir0, 'u_offset.mat'));
if counter==1
    load(append(dir0, 'botrace0.mat'));
    load(append(dir0, 'G2data.mat'));
    save(append(dir, 'G2data.mat'),'G2data')
    if LVswitch==0 %for code consistency
        load(append(dir0, 'perf_Data.mat'));
    end
    opt.resume_trace_data = botrace0;
    clear botrace0
    idx_G2=[];
    when_switch_s=[];
    save(append(dir, 'idx_G2.mat'),'idx_G2')
    save(append(dir, 'when_switch_s.mat'),'when_switch_s')
elseif counter>1
    load(append(dir, 'trace_file'))
    load(append(dir, 'G2data.mat'))
    load(append(dir, 'idx_G2.mat'))
    load(append(dir, 'when_switch_s.mat'))
    load(append(dir, 'LVgains'))
    opt.resume_trace_data = Trace;
    clear Trace
end
addpath("C:\mahdi\data_driven_controller")
if LVswitch==1 % means new exp_Data and perf_Data arrived from real system
    sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    ytmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
    utmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    counter_real= counter_real +1; % counter_real counts number of real system measurements
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
    perf_Data=[ov,Tr,st,ITAE];
    % add measured data to BO dataset
    J_measured=ObjFun(perf_Data);
    Trace=opt.resume_trace_data;
    values=Trace.values;
    values(end+1,1)=J_measured;
    Trace.values=values;
    samples=Trace.samples;
    samples = [samples; LVgains];
    Trace.samples = samples;
    save(append(dir, 'G2data.mat'),'G2data')
    save(append(dir, 'debug_G2data_',num2str(counter_real),'.mat'),'G2data')
    save(append(dir, 'perf_Data_',num2str(counter_real),'.mat'), 'perf_Data')
    save(append(dir, 'perf_Data_LV_',num2str(counter_real),'.mat'), 'perf_Data_LV')
    save(append(dir, 'exp_Data_',num2str(counter_real),'.mat'), 'exp_Data')
    LVswitch=0;
elseif LVswitch==0  % LVswitch==0 means we need to decide to call either real or surrogate to get data
    [~,~,Trace, LVgains,hyper_grid,idx_G2, G2, counter_s,when_switch_s, debug_ratio, ys] = LV_bayesoptGPML_v4_switch2(fun,opt,hyper_grid,counter_s, G2data,idx_G2,when_switch_s,counter_real);
    counter=counter+1; %counter: number of BO iteration in total
    consecutive_G2_counter=0; %to avoid dead loop on surrogate
    save(append(dir, 'debug_data_ys_',num2str(counter_real),'.mat'),'ys') %comment for switching 1
    save(append(dir, 'debug_data_debug_ratio_',num2str(counter_real),'.mat'),'debug_ratio') %comment for switching 1
    while counter_s>0 && consecutive_G2_counter<30
        consecutive_G2_counter=consecutive_G2_counter+1;
        save(append(dir, 'debug_G2_',num2str(counter_real),'_',num2str(idx_G2(end)),'.mat'), 'G2')
        opt.resume_trace_data = Trace;
        [~,~,Trace, LVgains,hyper_grid,idx_G2, G2, counter_s,when_switch_s, debug_ratio, ys] = LV_bayesoptGPML_v4_switch2(fun,opt,hyper_grid,counter_s, G2data,idx_G2,when_switch_s,counter_real);
        save(append(dir, 'idx_G2.mat'),'idx_G2')
        save(append(dir, 'when_switch_s.mat'),'when_switch_s')
        counter=counter+1; %counter: number of BO iteration in total
        save(append(dir, 'debug_data_ys_',num2str(counter_real),"_consecutive_G2_counter_",string(consecutive_G2_counter),'.mat'),'ys')  %comment for switching 1
        save(append(dir, 'debug_data_debug_ratio_',num2str(counter_real),"_consecutive_G2_counter_",string(consecutive_G2_counter),'.mat'),'debug_ratio')  %comment for switching 1
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
    if ~isempty(idx_G2)
        Trace.samples(idx_G2,:)=[];
        Trace.values(idx_G2)=[];
        Trace.post_mus(idx_G2)=[];
        Trace.post_sigma2s(idx_G2)=[];
        Trace.times(idx_G2)=[];
        Trace.AQ_vals(idx_G2)=[];
        save(append(dir, 'debug_idx_G2','.mat'),'idx_G2')
    end
    save(append(dir, 'trace_file_removed','.mat'),'Trace')
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