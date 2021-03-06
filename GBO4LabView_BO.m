% BO
global G2data
addpath("C:\Users\nobar\Documents\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\@iddata\iddata.m
dir0="C:\Users\nobar\Documents\LabVIEW Data\N0_Data\";
tmp_dir="C:\Users\nobar\Documents\LabVIEW Data\BO_Data\";
idName= 'demo_BO_0_2';
dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
withSurrogate=false;

N0=1; %for N0>1 modify
N_iter=50;
N_iter=N_iter+N0;

step_low=10;
step_high=20;
step_time=6;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

%% load gain limits
dir_gains=append('C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\tmp\DC_motor_gain_bounds\KpKi_bounds.mat');
load(dir_gains)

%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, sampleTf, sampleTs, npG2, N_G, N_G2_activated, N_perturbed);
else
    fun = @(perf_Data) ObjFun(perf_Data); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Setup the Gaussian Process (GP) Library
addpath("C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\gpml")
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
addpath("C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main")
[ms,mv,Trace_tmp, LVgains] = bayesoptGPML(fun,opt,N0, LVswitch, perf_Data(end,:));
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
save(append(dir, 'perf_Data_',num2str(counter)), 'perf_Data')
save(append(dir, 'exp_Data_',num2str(counter)), 'exp_Data')
counter=counter+1;

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gains0=[0.0950, 1.3293];
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