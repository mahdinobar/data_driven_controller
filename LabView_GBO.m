% GBO
addpath("C:\Users\nobar\Documents\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\@iddata\iddata.m
addpath("C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\tfest.m")
addpath("C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\")
dir0="C:\Users\nobar\Documents\LabVIEW Data\N0_Data_new_1\";
tmp_dir="C:\Users\nobar\Documents\LabVIEW Data\BO_Data\";
idName= 'demo_GBO_new_1';
dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
stat_value=60;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% counter=1;
% LVswitch=0;
% idx=0;
% exp_Data=0;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N0=1; %for N0>1 modify
N_iter=50;
N_extra=10;
N_iter=N_iter+N_extra+N0;
N_G=1;%N_G=5 number of iteration we use real plant before switching to surrogate G2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
N_G2_activated=15;
npG2=2;

sampleTf=2.5;%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% sampleTs=0.01;
% Nsample=sampleTf/sampleTs;
Nsample=150;
sampleTs=sampleTf/(Nsample-1);

step_low=80;
step_high=120;
step_time=4;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

%% load gain limits
dir_gains=append('C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\tmp\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
load(dir_gains)

%% We define the function we would like to optimize
fun = @(perf_Data) ObjFun(perf_Data); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.

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

%% to initialize first the response
if counter<1
    gains0=[0.5, 1.47]; %initial random
    Kp=gains0(1);
    Ki=gains0(2);
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
    G2data=G2data_init;
    save(append(dir, 'G2data.mat'),'G2data')
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

% perf_Data is only needed when LVswitch==1
[ms,mv,Trace_tmp, LVgains,hyper_grid_pruned] = bayesoptGPML(fun,opt,N0, LVswitch, mean(perf_Data(end-nr_repeats+1:end,:)),hyper_grid);

%     LVswitch==0 means we need to call the system to get data
if LVswitch==0
    if idx==N_G && N_G2_activated_counter<N_G2_activated
        if counter>1
            load(append(dir, 'G2data.mat'))
        end
        G2=tfest(G2data, npG2);
        C=tf([LVgains(1), LVgains(1)*LVgains(2)], [1, 0]);
        CL=feedback(C*G2, 1);
        ov=abs(stepinfo(CL).Overshoot);
        st=stepinfo(CL).SettlingTime;
        [y,t]=step(CL);
        reference=1;
        e=abs(y-reference);
        Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.6]).RiseTime;
        ITAE = trapz(t, t.*abs(e));
        perf_Data=[perf_Data;[ov, Tr, st, ITAE, 0]];
        save(append(dir, 'perf_Data_tmp'), 'perf_Data')
        LVswitch=1; % means bayesoptGPML will run completely
        [ms,mv,Trace_tmp, LVgains, hyper_grid_pruned] = bayesoptGPML(fun,opt,N0, LVswitch, [ov, Tr, st, ITAE, 0], hyper_grid);
        LVswitch=0;
        idx= 0;
        N_G2_activated_counter=N_G2_activated_counter+1;
        if counter>1
            % remove previous G2
            idx_G2= size(Trace_tmp.samples,1)-N_G-1;
            Trace_tmp.samples(idx_G2,:)=[];
            Trace_tmp.values(idx_G2)=[];
            Trace_tmp.post_mus(idx_G2)=[];
            Trace_tmp.post_sigma2s(idx_G2)=[];
            Trace_tmp.times(idx_G2)=[];
        end
        save(append(dir, 'G2_',num2str(counter)), 'G2')
    else

        Kp=LVgains(1);
        Ki=LVgains(2);
        gain_vel=Kp;
        Tn_vel=1/Ki;
        LVswitch=1;
        return
    end
elseif LVswitch==1 % means new exp_Data and perf_Data arrived
    sample_idx=exp_Data(:,3)==120; %pay attention!!!!!!!!!!!!!!!!!!!!!!!!!!
    ytmp = exp_Data(sample_idx,3);
    utmp= exp_Data(sample_idx,4);
    ytmp=ytmp(1:Nsample);
    utmp=utmp(1:Nsample);
    load(append(dir, 'G2data.mat'))
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    save(append(dir, 'G2data.mat'),'G2data')
    idx= idx +1; % idx counts number of real system after last G2

    LVswitch=0;
end

Trace(1)=Trace_tmp;
save(append(dir, 'trace_file.mat'),'Trace')
save(append(dir, 'perf_Data_',num2str(counter)), 'perf_Data')
save(append(dir, 'exp_Data_',num2str(counter)), 'exp_Data')
counter=counter+1;

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gains0=[0.0950, 3.3293];
Kp=gains0(1);
Ki=gains0(2);
gain_vel=Kp;
Tn_vel=1/Ki;
step_low=39;
Input_mode=2;
LVswitch=0;
counter=0;
return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%