% BO
global G2data
addpath("C:\Users\nobar\Documents\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\@iddata\iddata.m
dir0="C:\Users\nobar\Documents\LabVIEW Data\N0_Data\";
tmp_dir="C:\Users\nobar\Documents\LabVIEW Data\BO_Data\";
idName= 'demo_BO_0_1';
dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;

N0=1; %for N0>1 modify

step_low=10;
step_high=20;
step_time=6;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

load(append(dir, 'botrace0'), 'botrace0');
load(append(dir, 'G2data_init'),'G2data_init');


%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, sampleTf, sampleTs, npG2, N_G, N_G2_activated, N_perturbed);
else
    fun = @(X) ObjFun(X, G); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Setup the Gaussian Process (GP) Library
addpath("C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\gpml")
startup;
% Setting parameters for Bayesian Global Optimization
opt = defaultopt(); % Get some default values for non problem-specific options.
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
opt.resume_trace_data = botrace0;
clear botrace0
%%





return
