function run_solve(tag)
%
% Solve a computation:
%     - make the computation
%     - save the results
%
% The code should use (if possible) parfor loops (or any other parallel features).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('start solve: %s\n', tag)


% =========================================================================
% GPML toolbox based implementation
tmp_dir='/cluster/home/mnobar/GBO/GBO_1';
dir_gains=append('../', 'DC_motor_gain_bounds', '/', 'KpKi_bounds.mat');
% hyper-params
idName= 'results_1';
sys='DC_motor';
N0=10; %number of initial data
N_expr=2;

N_iter=50;
N_iter=N_iter+N0;
Nsample=150;
withSurrogate=false;
only_visualize=false;

if withSurrogate
    npG2=2;
    N_G2_activated=9999; %total number of times G2 is used
    N_G = 5; %number of consecutive optimization on real plant before surrogate
    N_extra= 10; % to compensate deleted iteration of surrogate(for N_G=2 use N_extra=40)
    N_iter=N_iter+N_extra;
end

dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end

%% load gain limits
load(dir_gains, 'Kp_min', 'Kp_max', 'Ki_min', 'Ki_max')

%% define plant
% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

%% only_visualize
if only_visualize
    load(append(dir, 'trace_file.mat'),'Trace')
    experiment=1;
    mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
    maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
    ms=[0 0];
    GBO_plots_one_experiment(ms, Trace, experiment, mins,maxes, N0, N_iter-N_extra, N_G, idName, G)
    return
end

%% Setup the Gaussian Process (GP) Library
addpath ../gpml/
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

%% find optimum GP hyperparameters (and initial data for first experiment)
% priors
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
% liklihood
likfunc={@likGauss};
% inference method
infer=@infExact;

% sample from latin (denoted as ltn) hypercube
N_ltn=N0;
RAND_ltn_all=zeros(N0,N_expr);

if withSurrogate==true
    load(append(dir,'RAND_ltn_all.mat'), 'RAND_ltn_all')
    RAND_ltn=RAND_ltn_all(:,1);
else
    RAND_ltn = sort(lhsdesign(N_ltn,1));
    RAND_ltn_all(:,1)=RAND_ltn;
    save(append(dir,'RAND_ltn_all.mat'))
end

Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
J_ltn = zeros(N_ltn,1);

sampleTf=1.5;
sampleTs=sampleTf/(Nsample-1);
global G2data

for i=1:N_ltn
    C=tf([Kp_ltn(i), Kp_ltn(i)*Ki_ltn(i)], [1, 0]);
    CL=feedback(C*G, 1);
    J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);

    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    %         todo check concept?
    if i==1
        G2data_init = iddata(ytmp,utmp,sampleTs);
    else
        G2data_init = merge(G2data_init, iddata(ytmp,utmp,sampleTs));
    end
end
G2data=G2data_init;
if withSurrogate
    G2=tfest(G2data, npG2);
end

%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, G2, sampleTf, sampleTs, npG2, N_G, N_G2_activated);
else
    fun = @(X) ObjFun(X, G); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%%
global N
global idx
global G2data
global N_G2_activated_counter

for expr=1:N_expr
    N=[];
    idx=[];
    G2_samples=[];
    G2_values=[];
    G2_post_mus=[];
    G2_post_sigma2s=[];
    % set initial dataset with latin hypercube samples
    % train data for GP
    X_ltn=[Kp_ltn, Ki_ltn];
    y_ltn=J_ltn;
    botrace.samples=X_ltn;
    botrace.values=y_ltn;
    % todo need to correct time?
    botrace.times=RAND_ltn';
    % save(append(dir,'trace_file.mat'),'botrace')
    opt.resume_trace_data = botrace;
    clear botrace

    for itr=N0+1:N_iter
        %     itr
        %     iteration=itr-N0

        opt.max_iters = size(opt.resume_trace_data.samples,1)+1;
        [ms,mv,Trace_tmp] = bayesoptGPML(fun,opt,N0);

        % remove previos data of older surrogate(G2) model, but keep them
        % seperately for plots
        if withSurrogate==true && N~=1 && idx==0
            G2_samples=[G2_samples; Trace_tmp.samples(end-N_G-1,:)];
            G2_values=[G2_values; Trace_tmp.values(end-N_G-1,:)];
            G2_post_mus=[G2_post_mus; Trace_tmp.post_mus(end-N_G-1,:)];
            G2_post_sigma2s=[G2_post_sigma2s; Trace_tmp.post_sigma2s(end-N_G-1,:)];
            Trace_tmp.samples(end-N_G-1,:)=[];
            Trace_tmp.values(end-N_G-1)=[];
            Trace_tmp.post_mus(end-N_G-1)=[];
            Trace_tmp.post_sigma2s(end-N_G-1)=[];
            Trace_tmp.times(end-N_G-1)=[];
        end
        opt.resume_trace_data = Trace_tmp;
        %     counter=counter+1;
    end

    % delete last trace of surrogate G2 for plots
    if withSurrogate==true && idx~=0
        G2_samples=[G2_samples; Trace_tmp.samples(end-idx,:)];
        G2_values=[G2_values; Trace_tmp.values(end-idx)];
        G2_post_mus=[G2_post_mus; Trace_tmp.post_mus(end-idx)];
        G2_post_sigma2s=[G2_post_sigma2s; Trace_tmp.post_sigma2s(end-idx)];
        Trace_tmp.samples(end-idx,:)=[];
        Trace_tmp.values(end-idx)=[];
        Trace_tmp.post_mus(end-idx)=[];
        Trace_tmp.post_sigma2s(end-idx)=[];
        Trace_tmp.times(end-idx)=[];
    end
    Trace_tmp.G2_samples=G2_samples;
    Trace_tmp.G2_values=G2_values;
    Trace_tmp.G2_post_mus=G2_post_mus;
    Trace_tmp.G2_post_sigma2s=G2_post_sigma2s;
    Trace(expr)=Trace_tmp;
    delete Trace_tmp

    if withSurrogate==true
        save(append(dir, 'trace_file.mat'),'Trace')
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
                ytmp=step(CL,0:sampleTs:sampleTf);
                utmp=step(CLU,0:sampleTs:sampleTf);
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
        save(append(dir,'RAND_ltn_all.mat'))
        Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
        Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
        J_ltn = zeros(N_ltn,1);
        for i=1:N_ltn
            J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);
        end
    end

end

%% Print results
fprintf('******************************************************\n');
fprintf('Best controller gains:      Kp=%2.4f, Ki=%2.4f\n',ms(1),ms(2));
fprintf('Associated cost: J([Kp,Ki])=%2.4f\n',mv);
fprintf('******************************************************\n');
% =========================================================================


% teardown
fprintf('end solve: %s\n', tag)
end


% =========================================================================
function [objective, constraints] = ObjFun(X, G)
%     todo move some lines outside with handler@: faster?
C=tf([X(1), X(1)*X(2)], [1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.6]).RiseTime;
ITAE = trapz(t, t.*abs(e));

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
constraints=-1;
end

function [objective] = ObjFun_Guided(X, G, G2, sampleTf, sampleTs, npG2, N_G,N_G2_activated)
global N
global idx
global G2data
global N_G2_activated_counter

if isempty(N)
    %     initially use G2
    N=1;
    objective=ObjFun(X, G2);
    idx= 0;
    N_G2_activated_counter=1;
elseif idx==N_G %&& N_G2_activated_counter<N_G2_activated
    N = N+1;
    %     G2idtf=idtf(n4sid(G2data,npG2));
    %     [a,b]=tfdata(G2idtf);
    %     G2=tf(a,b);
    G2=tfest(G2data, npG2);
    objective=ObjFun(X, G2);
    idx= 0;
    N_G2_activated_counter=N_G2_activated_counter+1;
else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    objective=ObjFun(X, G);
    C=tf([X(1),X(1)*X(2)], [1, 0]);
    CL=feedback(C*G, 1);
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
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
% =========================================================================
