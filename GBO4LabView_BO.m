idx=0;
N_G2_activated_counter=0;
% BO
global G2data
addpath("C:\mahdi\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\@iddata\iddata.m
dir0=append("C:\mahdi\LabVIEW Data\N0_Data_",string(expr),"\");
tmp_dir="C:\mahdi\LabVIEW Data\BO_Data\";
dir=append(tmp_dir,'\demo_BO_', string(expr), '\');
if not(isfolder(dir))
    mkdir(dir)
end

start_switch=1;
stat_value=60;
withSurrogate=false;

N0=1; %for N0>1 modify
N_iter=50;
N_iter=N_iter+N0;

step_low=80;
step_high=120;
step_time=5;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

%% load gain limits
dir_gains=append('C:\Users\students\Documents\data_driven_controller-main\data_driven_controller-main\tmp\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
load(dir_gains)

%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, sampleTf, sampleTs, npG2, N_G, N_G2_activated, N_perturbed);
else
    fun = @(perf_Data) ObjFun(perf_Data); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Setup the Gaussian Process (GP) Library
addpath("C:\Users\students\Documents\data_driven_controller-main\data_driven_controller-main\gpml")
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
if counter<1
    gains0_init=[0.5, 1.47]; %initial random
    % gains0=[0.4873, 1.5970]; %nominal for PM 90degree and GM=49db
    Kp=gains0_init(1);
    Ki=gains0_init(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
    Input_mode=2;
    start_switch=1;
    counter=counter+1;
    % Draw initial candidate grid from a Sobol sequence
    sobol = sobolset(opt.dims);
    hyper_grid_pruned = sobol(1:opt.grid_size,:);
    return
end

%%
% load initial dataset
if counter==1
    load(append(dir0, 'botrace0.mat'));
    load(append(dir0, 'G2data_init.mat'));
    if LVswitch==0 %for code consistency
        load(append(dir0, 'perf_Data.mat'));
    end
    opt.resume_trace_data = botrace0;
    clear botrace0
elseif counter>1
    load(append(dir, 'trace_file'))
    opt.resume_trace_data = Trace;
    clear Trace
end

opt.max_iters = size(opt.resume_trace_data.samples,1)+1;
addpath("C:\Users\students\Documents\data_driven_controller-main\data_driven_controller-main")
[ms,mv,Trace_tmp, LVgains, hyper_grid_pruned] = bayesoptGPML(fun,opt,N0, LVswitch, perf_Data, hyper_grid);
if counter==1 && LVswitch==1
    save(append(dir, 'test_trace_file.mat'),'Trace_tmp')
    save(append(dir, 'test_perf_Data_',num2str(counter)), 'perf_Data')
end
%     LVswitch==0 means we need to call the system to get data
if LVswitch==0
    Kp=LVgains(1);
    Ki=LVgains(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
    LVswitch=LVswitch+1;
    return
elseif LVswitch==1
    LVswitch=0;
end
Trace(1)=Trace_tmp;
save(append(dir, 'trace_file.mat'),'Trace')
save(append(dir, 'perf_Data_',num2str(counter),'_',num2str(expr)), 'perf_Data')
save(append(dir, 'exp_Data_',num2str(counter),'_',num2str(expr)), 'exp_Data')
counter=counter+1;

N_iterations=50;
if counter>N0+N_iterations
    expr=expr+1;
    counter=0;
    LVswitch=0;
end

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gains0=[0.0950, 3.3293];
Kp=gains0(1);
Ki=gains0(2);
gain_vel=Kp;
Tn_vel=1/Ki;
stat_value=0.5;
Input_mode=0;
LVswitch=0;
counter=0;
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

