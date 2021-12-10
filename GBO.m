function GBO
% GPML toolbox based implementation
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_GBO_1_5';
sys='DC_motor';
N0=3;
N_iter=30;
N_iter=N_iter+N0;
Nsample=50;
withSurrogate=true;
if withSurrogate
    npG2=2;
    N_G = 5; %number of consecutive optimization on real plant before surrogate
    N_extra= 6; % to compensate deleted iteration of surrogate
    N_iter=N_iter+N_extra;
end

dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end

%% define plant
% % robot-arm System ("Simultaneous computation of model order..., Badaruddin Muhammad et al.")
% num = [-0.0118, 0.0257, 0, 0, 0];
% den = [1, -3.1016, 4.3638, -3.1528, 1.0899, -0.0743];
% ts=1;
% G = d2c(tf(num,den, ts));

% DC motor at FHNW lab
num = [5.19908];
den = [1, 1.61335];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

%% load gain limits
if sys=="ball_screw"
    dir_gains=append(tmp_dir,'/', 'ball_screw_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="robot_arm"
    dir_gains=append(tmp_dir,'/', 'robot_arm_gain_bounds', '/', 'KpKiKd_bounds.mat');
elseif sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'KpKi_bounds.mat');
end
load(dir_gains)

%% create initial dataset
tmp=[];

% set random seed
rng('default')
rng(123)

% % initial values for GP of BO
RAND=rand(N0,1);

% load(append(dir,'RAND.mat'))

Kp = (Kp_max-Kp_min).*RAND + Kp_min;
Ki = (Ki_max-Ki_min).*RAND + Ki_min;
InitobjectiveData = zeros(N0,1);
% todo pay attention how you choose sampleTf?
sampleTf=1.5;
sampleTs=sampleTf/(Nsample-1);
global G2data
for i=1:N0
    C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
    CL=feedback(C*G, 1);
    InitobjectiveData(i) = ObjFun([Kp(i), Ki(i)], G);
    while isnan(InitobjectiveData(i)) || InitobjectiveData(i)>1000
        RAND(i)=rand(1,1);
        Kp(i) = (Kp_max-Kp_min).*RAND(i) + Kp_min;
        Ki(i) = (Ki_max-Ki_min).*RAND(i) + Ki_min;
        C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
        CL=feedback(C*G, 1);
        InitobjectiveData(i) = ObjFun(Kp(i), Ki(i), G);
    end
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    %         todo check concept?
    if i==1
        G2data = iddata(ytmp,utmp,sampleTs);
    else
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    end
end
clear ytmp
clear utmp
botrace.samples=[Kp, Ki];
botrace.values=InitobjectiveData;
% todo need to correct time?
botrace.times=RAND';

%% Setup the Gaussian Process (GP) Library
addpath ./gpml/
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

% save(append(dir,'trace_file.mat'),'botrace')
opt.resume_trace_data = botrace;
clear botrace
if withSurrogate
    G2 = tfest(G2data,npG2);
end

%% find optimum GP hyperparameters
% priors
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
% liklihood
likfunc={@likGauss};
% inference method
infer=@infExact;

% sample from latin (denoted as ltn) hypercube
N_ltn=25;
RAND_ltn = sort(lhsdesign(N_ltn,1));

Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
J_ltn = zeros(N_ltn,1);
for i=1:N_ltn
    C=tf([Kp_ltn(i), Kp_ltn(i)*Ki_ltn(i)], [1, 0]);
    CL=feedback(C*G, 1);
    J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);
end

N_hat=100;
RAND_hat = linspace(0,1,N_hat);
RAND_hat = RAND_hat(:);
Kp_hat = (Kp_max-Kp_min).*RAND_hat + Kp_min;
Ki_hat = (Ki_max-Ki_min).*RAND_hat + Ki_min;
J_hat = zeros(N_hat,1);
for i=1:N_hat
    C=tf([Kp_hat(i), Kp_hat(i)*Ki_hat(i)], [1, 0]);
    CL=feedback(C*G, 1);
    J_hat(i) = ObjFun([Kp_hat(i), Ki_hat(i)], G);
end

% train data for GP
X=[Kp_ltn, Ki_ltn];
y=J_ltn;

% test data x_hats for GP and ground truth y_hats
x_hats=[Kp_hat, Ki_hat];
y_hats=J_hat;

meanfunc = opt.meanfunc;
covfunc = opt.covfunc;
if isfield(opt,'num_mean_hypers')
    n_mh = opt.num_mean_hypers;
else
    n_mh = num_hypers(meanfunc{1},opt);
end
if isfield(opt,'num_cov_hypers')
    n_ch = opt.num_cov_hypers;
else
    n_ch = num_hypers(covfunc{1},opt);
end
hyp_latin = [];
hyp_latin.mean = zeros(n_mh,1);
hyp_latin.cov = zeros(n_ch,1);
hyp_latin.lik = log(0.1);
% calculate GP mean/cov/lik hyperparameters
hyp_latin = minimize(hyp_latin,@gp,-100,@infExact,meanfunc,covfunc,likfunc,X,y);

%     x_hats are test inputs given to gp to predict
[mu,sigma2] = gp(hyp_latin,infer,meanfunc,covfunc,likfunc,X,y,x_hats);

fig=figure();
fig.Position=[0 0 1600 1200];
subplot(2,1,1)
grid on
hold on
plot(X(:,1),y, 'r', 'LineWidth',3)
plot(x_hats(:,1),y_hats,'g', 'LineWidth',1)
plot(x_hats(:,1),mu,'k', 'LineWidth',3)
plot(x_hats(:,1),mu+sigma2/2,'--k', 'LineWidth',1)
plot(x_hats(:,1),mu-sigma2/2,'--k', 'LineWidth',1)
title(append('mean = ', func2str(opt.meanfunc{1}), ': ', num2str(hyp_latin.mean,'%05.3f'), ' & cov = ', func2str(opt.covfunc{1}), ' : ', ...
    num2str(hyp_latin.cov', '%05.3f'), ' & lik = ', func2str(likfunc{1}), ' : ', num2str(hyp_latin.lik,'%05.3f')))
xlabel('Kp')
ylabel('cost')
legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
xlim([Kp_min, Kp_max])
subplot(2, 1, 2)
grid on
hold on
plot(X(:,2),y, 'r', 'LineWidth',3)
plot(x_hats(:,2),y_hats,'g', 'LineWidth',1)
plot(x_hats(:,2),mu,'k', 'LineWidth',3)
plot(x_hats(:,2),mu+sigma2/2,'--k', 'LineWidth',1)
plot(x_hats(:,2),mu-sigma2/2,'--k', 'LineWidth',1)
legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
xlabel('Ki')
ylabel('cost')
legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
xlim([Ki_min, Ki_max])

figName=append(dir, idName,'_GP_hypr_tune_matern5.png');
saveas(gcf,figName)
pause;
close;

%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, G2, sampleTf, sampleTs, npG2, N_G);
else
    fun = @(X) ObjFun(X, G); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end
%% plot true J (grid)
% Let's plot grid of points just to see what we are trying to optimize
clf;
Kp_range=Kp_max-Kp_min;
resol=15;
Kp_surf_resol=Kp_range/resol;
Ki_range=Ki_max-Ki_min;
Ki_surf_resol=Ki_range/resol;
[kp_pt,ki_pt]=meshgrid(Kp_min:Kp_surf_resol:Kp_max,Ki_min:Ki_surf_resol:Ki_max);
j_pt=zeros(size(kp_pt));
c_pt=zeros(size(kp_pt));
for i=1:size(kp_pt,1)
    for j=1:size(kp_pt,2)
        [l,c]=ObjFun([kp_pt(i,j),ki_pt(i,j)],G);
        j_pt(i,j)=l;
        c_pt(i,j)=c;
    end
end
j_pt(c_pt>opt.lt_const)=NaN;
surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
xlabel('Kp')
ylabel('Ki')
zlabel('J')
drawnow;

%% Start the optimization
fprintf('Optimizing hyperparamters of function "samplef.m" ...\n');
% [ms,mv,Trace] = bayesoptGPML(fun,opt);   % ms - Best parameter setting found
% mv - best function value for that setting L(ms)
% Trace  - Trace of all settings tried, their function values, and constraint values.

%%
global N
global idx
N=[];
idx=[];
counter=N0+1;
G2_samples=[];
G2_values=[];
G2_post_mus=[];
G2_post_sigma2s=[];
for itr=N0+1:N_iter
    %     itr
    %     iteration=itr-N0

    opt.max_iters = size(opt.resume_trace_data.samples,1)+1;
    [ms,mv,Trace] = bayesoptGPML(fun,opt,hyp_latin);

    % remove previos data of older surrogate(G2) model, but keep them
    % seperately for plots
    if withSurrogate==true && N~=1 && idx==0
        G2_samples=[G2_samples; Trace.samples(end-N_G-1,:)];
        G2_values=[G2_values; Trace.values(end-N_G-1,:)];
        G2_post_mus=[G2_post_mus; Trace.post_mus(end-N_G-1,:)];
        G2_post_sigma2s=[G2_post_sigma2s; Trace.post_sigma2s(end-N_G-1,:)];
        Trace.samples(end-N_G-1,:)=[];
        Trace.values(end-N_G-1)=[];
        Trace.post_mus(end-N_G-1)=[];
        Trace.post_sigma2s(end-N_G-1)=[];
        Trace.times(end-N_G-1)=[];
    end
    opt.resume_trace_data = Trace;
    %     counter=counter+1;
end
% delete last trace of surrogate G2 for plots
if withSurrogate==true && idx~=0
    G2_samples=[G2_samples; Trace.samples(end-idx,:)];
    G2_values=[G2_values; Trace.values(end-idx)];
    G2_post_mus=[G2_post_mus; Trace.post_mus(end-idx)];
    G2_post_sigma2s=[G2_post_sigma2s; Trace.post_sigma2s(end-idx)];
    Trace.samples(end-idx,:)=[];
    Trace.values(end-idx)=[];
    Trace.post_mus(end-idx)=[];
    Trace.post_sigma2s(end-idx)=[];
    Trace.times(end-idx)=[];
end
Trace.G2_samples=G2_samples;
Trace.G2_values=G2_values;
Trace.G2_post_mus=G2_post_mus;
Trace.G2_post_sigma2s=G2_post_sigma2s;

% save(append(dir, 'trace_file_BO.mat'),'Trace')

%% Print results
fprintf('******************************************************\n');
fprintf('Best controller gains:      Kp=%2.4f, Ki=%2.4f\n',ms(1),ms(2));
fprintf('Associated cost: J([Kp,Ki])=%2.4f\n',mv);
fprintf('******************************************************\n');

%% Draw optimium
hold on;
plot3([ms(1) ms(1)],[ms(2) ms(2)],[max(j_pt(:)) min(j_pt(:))],'r-','LineWidth',2);

%% plots
if withSurrogate
    save(append(dir, 'trace_file.mat'),'Trace')
    GBO_plots(ms, mv, Trace, opt.mins, opt.maxes, N0, N_iter-N_extra, N_G, idName, G)
else
    save(append(dir, 'trace_file_BO.mat'),'Trace')
    %     GBO_plots(ms, mv, Trace, opt.mins, opt.maxes, N0, N_iter, N_G, idName, G)
end
end

function [objective, constraints] = ObjFun(X, G)

%     todo move some lines outside with handler@: faster?
C=tf([X(1), X(1)*X(2)], [1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.98]).RiseTime;
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


function [objective] = ObjFun_Guided(X, G, G2, sampleTf, sampleTs, npG2, N_G)
global N
global idx
global G2data
if isempty(N)
    N=1;
    objective=ObjFun(X, G2);
    idx= 0;
elseif idx==N_G
    N = N+1;
    G2 = tfest(G2data,npG2);
    objective=ObjFun(X, G2);
    idx= 0;
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
        else
            error('bayesopt:unkhyp','Unknown number of hyperparameters asked for by one of the functions');
        end
    end
end