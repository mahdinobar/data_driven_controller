global G2data

stat_value=0.5;
step_low=10;
step_high=20;
step_time=6;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

if counter==0
Kp=0.0950;
Ki=1.3293;
gain_vel=Kp;
Tn_vel=1/Ki;
counter=counter+1;
return
end

sampleTs=1/100;
if counter==1
    Kp=0.0950;
    Ki=1.3293;
    ytmp = exp_Data(:,3);
    utmp= exp_Data(:,4);
    G2data_init = iddata(ytmp,utmp,sampleTs);
    J_init=ObjFun(exp_Data(end,:));
    botrace.samples=[Kp, Ki];
    botrace.values=J_init;
    botrace.times=0;
    return
save('C:\Users\nobar\Documents\LabVIEW Data\counter.mat','counter');
save('C:\Users\nobar\Documents\LabVIEW Data\perf_Data.mat','perf_Data');
save('C:\Users\nobar\Documents\LabVIEW Data\exp_Data.mat','exp_Data');
end


%%
tmp_dir='C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\tmp';
% hyper-params
idName= 'demo_GBO_0_1';
sys='DC_motor';
N0=1; %number of initial data
N_expr=1;

N_iter=50;
N_iter=N_iter+N0;
sampleTf=1.5;
Nsample=150;
eps=0.0;
N_perturbed=1; % number of perturbed plus one not perturbed surrogate
withSurrogate=false;
only_visualize=false;

dir=append(tmp_dir,'\', idName, '\');
if not(isfolder(dir))
    mkdir(dir)
end

%% load gain limits
if sys=="ball_screw"
    dir_gains=append(tmp_dir,'/', 'ball_screw_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="robot_arm"
    dir_gains=append(tmp_dir,'/', 'robot_arm_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="DC_motor"
    dir_gains=append(tmp_dir,'\', 'DC_motor_gain_bounds', '\', 'KpKi_bounds.mat');
end
load(dir_gains)

%% Setup the Gaussian Process (GP) Library
addpath C:\Users\nobar\Documents\data_driven_controller-main\data_driven_controller-main\gpml\
startup;

% Setting parameters for Bayesian Global Optimization
opt = defaultopt(); % Get some default values for non problem-specific options.
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
opt.grid_size = 20000;
%opt.parallel_jobs = 3; % Run 3 jobs in parallel using the approach in (Snoek et al., 2012). Increases overhead of BO, so probably not needed for this simple function.
opt.lt_const = 0.0;
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

%% find optimum GP hyperparameters (and initial data for first experiment)
% priors
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
% liklihood
likfunc={@likGauss};
% inference method
infer=@infExact;

%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, sampleTf, sampleTs, npG2, N_G, N_G2_activated, N_perturbed);
else
    fun = @(X) ObjFun(X, LVswitch); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Start the optimization
fprintf('Optimizing hyperparamters of function "samplef.m" ...\n');

%%
global N
global idx
global G2data
global N_G2_activated_counter
global N_pr
global expr_G2rmse

G2rmse=[];

expr_G2rmse=[];
fprintf('>>>>>experiment: %d \n', expr);
N=0;
idx=[];
N_pr=0;
G2_samples=[];
G2_values=[];
G2_post_mus=[];
G2_post_sigma2s=[];
% set initial dataset with latin hypercube samples
% train data for GP
X_ltn=[Kp_ltn, Ki_ltn];
y_ltn=J_ltn;
opt.resume_trace_data = botrace;
clear botrace
idx_G2=[];
for itr=N0+1:N_iter
    opt.max_iters = size(opt.resume_trace_data.samples,1)+1;
    [ms,mv,Trace_tmp] = bayesoptGPML(fun,opt,N0);
    % remove previos data of older surrogate(G2) model, but keep them
    % seperately for plots
    if withSurrogate==true && N>N_perturbed && idx==0
        for i=1:1:N_perturbed
            G2_samples=[G2_samples; Trace_tmp.samples(end-N_G-i,:)];
            G2_values=[G2_values; Trace_tmp.values(end-N_G-i,:)];
            G2_post_mus=[G2_post_mus; Trace_tmp.post_mus(end-N_G-i,:)];
            G2_post_sigma2s=[G2_post_sigma2s; Trace_tmp.post_sigma2s(end-N_G-i,:)];
            idx_G2= [idx_G2;size(Trace_tmp.samples,1)-N_G-i];
        end
    end
    opt.resume_trace_data = Trace_tmp;
end

Trace_tmp.G2_samples=G2_samples;
Trace_tmp.G2_values=G2_values;
Trace_tmp.G2_post_mus=G2_post_mus;
Trace_tmp.G2_post_sigma2s=G2_post_sigma2s;

Trace_tmp.samples(idx_G2,:)=[];
Trace_tmp.values(idx_G2)=[];
Trace_tmp.post_mus(idx_G2)=[];
Trace_tmp.post_sigma2s(idx_G2)=[];
Trace_tmp.times(idx_G2)=[];
Trace(expr)=Trace_tmp;
delete Trace_tmp
if withSurrogate==true
    save(append(dir, 'trace_file.mat'),'Trace')
    save(append(dir, 'idx_G2.mat'),'idx_G2')
    if expr<N_expr
        load(append(dir,'RAND_ltn_all.mat'), 'RAND_ltn_all')
        RAND_ltn=RAND_ltn_all(:,expr+1);
        Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
        Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
        J_ltn = zeros(N_ltn,1);
        for i=1:N_ltn
            C=tf([Kp_ltn(i), Kp_ltn(i)*Ki_ltn(i)], [1, 0]);
            CL=feedback(C*G, 1);
            J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);
            CLU=feedback(C, G);
            ytmp=step(CL,eps:sampleTs:sampleTf);
            utmp=step(CLU,eps:sampleTs:sampleTf);
            %         todo check concept?
            if i==1
                G2data_init = iddata(ytmp,utmp,sampleTs);
            else
                G2data_init = merge(G2data_init, iddata(ytmp,utmp,sampleTs));
            end
        end
        G2data=G2data_init;
    end
else
    save(append(dir, 'trace_file_BO.mat'),'Trace')
    RAND_ltn = sort(lhsdesign(N_ltn,1));
    RAND_ltn_all(:,expr+1)=RAND_ltn;
    save(append(dir,'RAND_ltn_all.mat'),'RAND_ltn_all')
    Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
    Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
    J_ltn = zeros(N_ltn,1);
    for i=1:N_ltn
        J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);
    end
end
if withSurrogate
    G2rmse=[G2rmse, expr_G2rmse];
    save(append(dir, 'G2rmse.mat'),'G2rmse')
end









counter=counter+1;



%% functions
function [objective] = ObjFun(perf_Data, LVswitch)

if ~LVswitch
    pass
else
ov=abs(perf_Data(1));
st=perf_Data(3);
Tr=perf_Data(2);
ITAE = perf_Data(4);

if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end

if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=1e5;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=1e5;
end

w=[0.1, 1, 1, 0.5];
w=w./sum(w);
objective=ov/w(1)+st/w(2)+Tr/w(3)+ITAE/w(4);
end
end
