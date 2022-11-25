% GPML toolbox based implementation
% version 5
function GBO_v5
%% clean start, set directories
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
idName= 'demo_GBO_v5_0_4';
sys='DC_motor';
dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end

%% set hyperparameters
isGBO=false;
objective_noise=false;
N0=1; %number of initial data
N_expr=2;
N_iter=50;
N_iter=N_iter+N0;
Nsample=150;
sampleTf=2.5; %based on the min and max settling time equal to 1.3 and 19 seconds inside the feasible set "KpKi_bounds_new_2.mat" we choose 1.5 for DC motor plant with speed sensor pole 9.918e-5
sampleTs=sampleTf/(Nsample-1);
sampleTinit=0.0;
lt_const=0.0;
initRant="latin"; %build initial set randomnly witith latin hypercubes
% uncomment for isGBO
npG2=2;

%% define plant
% DC motor at FHNW lab
% speed sensor pole 9.918e-5
num = [9.54434];
den = [1, 4.14479, 4.19941];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);

%% load gain limits (feasible set)
if sys=="DC_motor"
    dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'KpKi_bounds_new_2.mat');
end
load(dir_gains)

%% build initial dataset (N0)
if initRant=="latin"
    % latin hypercube samples
    if isGBO
        % load same samples used for BO
        load(append(dir,'RAND_ltn_all.mat'), 'RAND_all_expr')
    else
        % sample from latin (denoted as ltn) hypercube
        RAND_all_expr=zeros(N0,N_expr);
        for expr=1:N_expr
            RAND = sort(lhsdesign(N0,1));
            RAND_all_expr(:,expr)=RAND;
        end
        save(append(dir,'RAND_ltn_all.mat'),'RAND_all_expr')
    end
end

%% plot true J (grid)
% % uncomment for adjusting weights (debug)
% global data_tmp
% data_tmp=[];
clf;
Kp_range=Kp_max-Kp_min;
resol=25;
Kp_surf_resol=Kp_range/resol;
Ki_range=Ki_max-Ki_min;
Ki_surf_resol=Ki_range/resol;
[kp_pt,ki_pt]=meshgrid(Kp_min:Kp_surf_resol:Kp_max,Ki_min:Ki_surf_resol:Ki_max);
j_pt=zeros(size(kp_pt));
c_pt=zeros(size(kp_pt));
for i=1:size(kp_pt,1)
    for j=1:size(kp_pt,2)
        [l,c]=ObjFun([kp_pt(i,j),ki_pt(i,j)],G, false);
        j_pt(i,j)=l;
        c_pt(i,j)=c;
    end
end
j_pt(c_pt>lt_const)=NaN;
surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
xlabel('Kp')
ylabel('Ki')
zlabel('J')
set(gca,'zscale','log')
set(gca,'ColorScale','log')
save(append(dir, 'grount_truth.mat'),'j_pt','kp_pt','ki_pt')

%% plot optimum (ground truth by grid search)
% ground truth grid search optimum
[J_gt,I]=min(j_pt,[],'all');
hold on;
plot3([kp_pt(I) kp_pt(I)],[ki_pt(I) ki_pt(I)],[max(j_pt(:)) min(j_pt(:))],'g-','LineWidth',3);

%% Setup the Gaussian Process (GP) Library
addpath ./gpml/
startup;
% Setting parameters for Bayesian Global Optimization
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};

opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
opt.grid_size = 20000;
%opt.parallel_jobs = 3; % Run 3 jobs in parallel using the approach in (Snoek et al., 2012). Increases overhead of BO, so probably not needed for this simple function.
opt.lt_const = lt_const;
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

%% We define the function we would like to optimize
if isGBO==true
    fun = @(X, surrogate)ObjFun_Guided_v5(X, surrogate, G, sampleTf, sampleTs, npG2, sampleTinit, objective_noise);
else
    fun = @(X) ObjFun(X, G, objective_noise); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end

%% Start the optimization
global N
global idx
global G2data
global N_G2_tmp
global expr_G2rmse
% each experiment is the entire iterations starting with certain initial set
for expr=1:1:N_expr
    expr_G2rmse=[];
    fprintf('>>>>>experiment: %d \n', expr);
    N=0;
    idx=[];
    N_G2_tmp=0;
    G2_samples=[];
    G2_values=[];
    G2_post_mus=[];
    G2_post_sigma2s=[];
    % create initial dataset per experiment
    RAND=RAND_all_expr(:,expr);
    Kp_ltn = (Kp_max-Kp_min).*RAND + Kp_min;
    Ki_ltn = (Ki_max-Ki_min).*RAND + Ki_min;
    J_ltn = zeros(N0,1);
    for i=1:N0
        C=tf([Kp_ltn(i), Kp_ltn(i)*Ki_ltn(i)], [1, 0]);
        CL=feedback(C*G, 1);
        J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G, objective_noise);
        if isGBO==true
            CLU=feedback(C, G);
            ytmp=step(CL,sampleTinit:sampleTs:sampleTf);
            utmp=step(CLU,sampleTinit:sampleTs:sampleTf);
            if objective_noise==true
                noise_y = (mean(ytmp)*5/100)*randn(length(ytmp),1);
                noise_u = (mean(utmp)*5/100)*randn(length(utmp),1);
                ytmp=ytmp+noise_y;
                utmp=utmp+noise_u;
            end
            if i==1
                G2data = iddata(ytmp,utmp,sampleTs);
            else
                G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
            end
        end
    end
    % set initial dataset
    X_ltn=[Kp_ltn, Ki_ltn];
    y_ltn=J_ltn;
    botrace.samples=X_ltn;
    botrace.values=y_ltn;
    % todo need to correct time?
    botrace.times=RAND';
    opt.resume_trace_data = botrace;
    clear botrace
    % todo check concept of max_iters?
    opt.max_iters = size(opt.resume_trace_data.samples,1)+N_iter-1;
    [ms,mv,Trace(expr)] = bayesoptGPML_v5(fun,opt,N0, isGBO);
    if isGBO==true
        save(append(dir, 'trace_file.mat'),'Trace')
        save(append(dir, 'G2rmse_', num2str(expr),'.mat'),'expr_G2rmse')
    else
        save(append(dir, 'trace_file_BO.mat'),'Trace')
    end
end

%% Draw optimium
hold on;
plot3([ms(1) ms(1)],[ms(2) ms(2)],[max(j_pt(:)) min(j_pt(:))],'r-','LineWidth',2);

if isGBO
    figName=append(dir, idName,'_SurfGrid_GBO_Solution.png');
else
    figName=append(dir, idName,'_SurfGrid_BO_Solution.png');
end
saveas(gcf,figName)

end

function [objective, constraints] = ObjFun(X, G, objective_noise)
% todo move some lines outside with handler@: faster?
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
% % uncomment for adjusting weights (debug)
% global data_tmp
% data_tmp=[data_tmp;[ov, st, Tr, ITAE]];
% w=[2, 1, 1, 0.5];
% w=[1, 0.12, 1, 0.5];
w_mean_grid=[10.5360, 3.8150, 0.6119, 1.1596];
w_importance=[2, 1, 1, 1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);
if objective_noise==true
    noise = (objective*5/100)*randn(1,1);  % gives you 1000 samples
    objective=objective+noise;
end
constraints=-1;
% if isnan(ov) || isinf(ov) || ov>1e3 ...
%         || isnan(st) || isinf(st) || st>1e3 ...
%         || isnan(Tr) || isinf(Tr) || Tr>1e3 ...
%         || isnan(ITAE) || isinf(ITAE) || ITAE>1e3
%     objective=1e3;
% end
end

function [objective, N_G2_tmp] = ObjFun_Guided_v5(X, surrogate, G, sampleTf, sampleTs, npG2, sampleTinit, objective_noise)
global N
global G2data
global N_G2_tmp
global expr_G2rmse

N=N+1;
if surrogate==true
    G2=tfest(G2data, npG2);
    objective=ObjFun(X, G2, false);
    t=0:3/100:3;
    y = step(G,t);
    y2 = step(G2,t);
    rmse2=sqrt(mean((y-y2).^2));
    expr_G2rmse=[expr_G2rmse;rmse2];
    N_G2_tmp=N_G2_tmp+1;
elseif surrogate==false
    N_G2_tmp=0;
    objective=ObjFun(X, G, objective_noise);
    C=tf([X(1),X(1)*X(2)], [1, 0]);
    CL=feedback(C*G, 1);
    CLU=feedback(C, G);
    ytmp=step(CL,sampleTinit:sampleTs:sampleTf);
    utmp=step(CLU,sampleTinit:sampleTs:sampleTf);
    if objective_noise==true
        noise_y = (mean(ytmp)*5/100)*randn(length(ytmp),1);
        noise_u = (mean(utmp)*5/100)*randn(length(utmp),1);
        ytmp=ytmp+noise_y;
        utmp=utmp+noise_u;
    end
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
end
fprintf('N= %d \n', N);
fprintf('N_G2_tmp= %d \n', N_G2_tmp);
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