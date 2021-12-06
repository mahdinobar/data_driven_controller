function GBO
% GPML toolbox based implementation
clear; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_GBO_0_6';
sys='DC_motor';
N0=3;
N_iter=30;
N_iter=N_iter+N0;
Nsample=50;
withSurrogate=true;
if withSurrogate
    npG2=2;
    N_G = 5; %number of consecutive optimization on real plant before surrogate
    N_extra= 5; % to compensate deleted iteration of surrogate
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
opt.save_trace = 1;
%opt.trace_file = 'demo_trace.mat';
%matlabpool 3; % Uncomment to do certain things in parallel. Suggested if optimize_ei is turned on. If parallel_jobs is > 1, bayesopt does this for you.
opt.trace_file=append(dir,'trace_file.mat');
opt.resume_trace=true;

% save(append(dir,'trace_file.mat'),'botrace')
opt.resume_trace_data = botrace;
clear botrace

G2 = tfest(G2data,npG2);
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
for itr=N0+1:N_iter
%     itr
%     iteration=itr-N0
    
    opt.max_iters = size(opt.resume_trace_data.samples,1)+1;
    [ms,mv,Trace] = bayesoptGPML(fun,opt);

    %         remove previos data of older surrogate(G2) model
    if withSurrogate==true && rem(itr-N0-1,N_G+1)==0 && itr>N0+1
        fprintf('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n');
        itr
        Trace.samples(counter-N_G-1,:)=[];
        Trace.values(counter-N_G-1)=[];
        Trace.times(counter-N_G-1)=[];
        counter=counter-1;
    end
    opt.resume_trace_data = Trace;
    counter=counter+1;
end

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
    GBO_plots(ms, mv, Trace, opt.mins, opt.maxes, N0, N_iter-N_extra, idName, G)
else
    GBO_plots(ms, mv, Trace, opt.mins, opt.maxes, N0, N_iter, idName, G)
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

w1=0.1;
w2=1;
w3=1;
w4=0.5;
objective=ov/w1+st/w2+Tr/w3+ITAE/w4;
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