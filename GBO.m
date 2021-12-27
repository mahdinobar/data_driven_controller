function GBO
% GPML toolbox based implementation
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_GBO_0_45';
sys='DC_motor';
N0=1; %number of initial data
N_expr=3;

N_iter=50;
N_iter=N_iter+N0;
% surrogate final time is sufficient(??) to cover settling time of closed
% loop instead (but more)
sampleTf=1.5;
Nsample=150;
eps=0.0;
withSurrogate=false;
only_visualize=false;

if withSurrogate
    npG2=2;
    N_G2_activated=5; %total number of times G2 is used
    N_G = 1; %number of consecutive optimization on real plant before surrogate
    N_extra= N_G2_activated; %use (N_G2_activated) if you use N_G2_activated;  to compensate deleted iteration of surrogate(for N0=10, N_G=2 use N_extra=27)
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
% Kp_min=Kp_min-15;
% Kp_max=Kp_max+10;
% Ki_min=0.1;

%% only_visualize
if only_visualize
    load(append(dir, 'trace_file.mat'),'Trace')
    
    experiment=1;
    mins = [Kp_min, Ki_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
    maxes = [Kp_max, Ki_max]; % Vector of maximum values for each parameter.
    val_tmp=(Trace.values);
    [mv_tmp,mi_tmp] = min(val_tmp);
    ms=Trace(experiment).samples(mi_tmp,:);
    GBO_plots_all_experiments(Trace, N0, N_iter-N_extra, idName)
    GBO_plots_one_experiment(ms, Trace, experiment, mins,maxes, N0, N_iter-N_extra, N_G, idName, G)
    return
end
%% create initial dataset
% tmp=[];
%
% % set random seed
% rng('default')
% rng(123)
%
% % % initial values for GP of BO
% RAND=rand(N0,1);
%
% % load(append(dir,'RAND.mat'))
%
% Kp = (Kp_max-Kp_min).*RAND + Kp_min;
% Ki = (Ki_max-Ki_min).*RAND + Ki_min;
% InitobjectiveData = zeros(N0,1);
% % todo pay attention how you choose sampleTf?
% sampleTf=0.5;
% sampleTs=sampleTf/(Nsample-1);
% global G2data
% for i=1:N0
%     C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
%     CL=feedback(C*G, 1);
%     InitobjectiveData(i) = ObjFun([Kp(i), Ki(i)], G);
%     while isnan(InitobjectiveData(i)) || InitobjectiveData(i)>1000
%         RAND(i)=rand(1,1);
%         Kp(i) = (Kp_max-Kp_min).*RAND(i) + Kp_min;
%         Ki(i) = (Ki_max-Ki_min).*RAND(i) + Ki_min;
%         C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
%         CL=feedback(C*G, 1);
%         InitobjectiveData(i) = ObjFun(Kp(i), Ki(i), G);
%     end
%     CLU=feedback(C, G);
%     ytmp=step(CL,eps:sampleTs:sampleTf);
%     utmp=step(CLU,eps:sampleTs:sampleTf);
%     %         todo check concept?
%     if i==1
%         G2data = iddata(ytmp,utmp,sampleTs);
%     else
%         G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
%     end
% end
% clear ytmp
% clear utmp
% botrace.samples=[Kp, Ki];
% botrace.values=InitobjectiveData;
% % todo need to correct time?
% botrace.times=RAND';

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

% final simulation sampling time
sampleTs=sampleTf/(Nsample-1);
global G2data

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
if withSurrogate
%     G2_tmp=n4sid(G2data,npG2);
%     G2idtf=idtf(G2_tmp);
%     [a,b]=tfdata(G2idtf);
%     G2=tf(a,b);
    G2=tfest(G2data, npG2);
    t=0:1/100:3.3;
    y = step(G,t);
    y2 = step(G2,t);

    % %     uncomment to check simulation
    figure(1)
    step(G); hold on; step(G2,'r')
    rmse2=sqrt(mean((y-y2).^2))
    
%     close
    figure(2);
    compare(G2data, G2)
%     close
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
X_ltn=[Kp_ltn, Ki_ltn];
y_ltn=J_ltn;

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
hyp_latin = minimize(hyp_latin,@gp,-100,@infExact,meanfunc,covfunc,likfunc,X_ltn,y_ltn);

%     x_hats are test inputs given to gp to predict
[mu,sigma2] = gp(hyp_latin,infer,meanfunc,covfunc,likfunc,X_ltn,y_ltn,x_hats);
save(append(dir, 'hyp_latin.mat'), 'hyp_latin')

% %% plot latin tuning GP hyperparams
% fig=figure();
% fig.Position=[0 0 1600 1200];
% subplot(2,1,1)
% grid on
% hold on
% plot(X_ltn(:,1),y_ltn, 'r', 'LineWidth',3)
% plot(x_hats(:,1),y_hats,'g', 'LineWidth',1)
% plot(x_hats(:,1),mu,'k', 'LineWidth',3)
% plot(x_hats(:,1),mu+sigma2/2,'--k', 'LineWidth',1)
% plot(x_hats(:,1),mu-sigma2/2,'--k', 'LineWidth',1)
% % title(append('mean = ', func2str(opt.meanfunc{1}), ': ' ...
% %     , num2str(hyp_latin.mean,'%05.3f'), ' & cov = ', func2str(opt.covfunc{1}) ...
% %     , ' : ', num2str(hyp_latin.cov', '%05.3f'), ' & lik = ', ...
% %     func2str(likfunc{1}), ' : ', num2str(hyp_latin.lik,'%05.3f')))
% title(append('mean = ', func2str(opt.meanfunc{1}), ' & cov = ', ...
%     func2str(opt.covfunc{1}), ' & lik = ', func2str(likfunc{1})))
% xlabel('Kp')
% ylabel('cost')
% legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
% xlim([Kp_min, Kp_max])
% ylim([0, max(y_hats)+10])
% subplot(2, 1, 2)
% grid on
% hold on
% plot(X_ltn(:,2),y_ltn, 'r', 'LineWidth',3)
% plot(x_hats(:,2),y_hats,'g', 'LineWidth',1)
% plot(x_hats(:,2),mu,'k', 'LineWidth',3)
% plot(x_hats(:,2),mu+sigma2/2,'--k', 'LineWidth',1)
% plot(x_hats(:,2),mu-sigma2/2,'--k', 'LineWidth',1)
% legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
% xlabel('Ki')
% ylabel('cost')
% legend('training samples', 'test data', 'posterior mean', 'posterior confidence bound')
% xlim([Ki_min, Ki_max])
% ylim([0, max(y_hats)+10])
% figName=append(dir, idName,'_GP_hypr_tune_matern5.png');
% saveas(gcf,figName)
% pause(1.5);
% close;
% 
%% We define the function we would like to optimize
if withSurrogate==true
    fun = @(X)ObjFun_Guided(X, G, G2, sampleTf, sampleTs, npG2, N_G, N_G2_activated);
else
    fun = @(X) ObjFun(X, G); % CBO needs a function handle whose sole parameter is a vector of the parameters to optimize over.
end
%% plot true J (grid)
% Let's plot grid of points just to see what we are trying to optimize
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
set(gca,'zscale','log')
set(gca,'ColorScale','log')
% ground truth grid search optimum
[J_gt,I]=min(j_pt,[],'all')
% hold on;
% plot3([kp_pt(I) kp_pt(I)],[ki_pt(I) ki_pt(I)],[max(j_pt(:)) min(j_pt(:))],'g-','LineWidth',3);
% Kp_nominal=50.3549;
% Ki_nominal=1.6134;
% J_nominal=ObjFun([Kp_nominal, Ki_nominal],G);
% % optimality ratio of nominal gains
% OR_nominal=J_nominal/J_gt
% plot3([Kp_nominal Kp_nominal],[Ki_nominal Ki_nominal],[max(j_pt(:)) min(j_pt(:))],'k-','LineWidth',3);
% 
% % % uncomment to inspect for finding sampleTf
% % C=tf([Kp_max, Kp_max*Ki_max], [1, 0]);
% % CL=feedback(C*G, 1);
% % figure()
% % step(CL)
% % step(CLU)
% 
% % zlim([0,50])
% % [true_objective, b]=min(j_pt,[],'all');
% % kp_true=kp_pt(b)
% % ki_true=ki_pt(b)
drawnow;

%% Start the optimization
fprintf('Optimizing hyperparamters of function "samplef.m" ...\n');
% [ms,mv,Trace] = bayesoptGPML(fun,opt);   % ms - Best parameter setting found
% mv - best function value for that setting L(ms)
% Trace  - Trace of all settings tried, their function values, and constraint values.

%%
global N
global idx
global G2data
global N_G2_activated_counter

expr=1;
while expr<N_expr+1
    fprintf('>>>>>experiment: %d \n', expr);
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
    opt.resume_trace_data = botrace;
    clear botrace
    idx_G2=[];
    for itr=N0+1:N_iter

        opt.max_iters = size(opt.resume_trace_data.samples,1)+1;

        [ms,mv,Trace_tmp] = bayesoptGPML(fun,opt,N0);
% %         TODO simply for BO repeat the random selection of initial gain with new experiment but for GBO ignore if an experiment fails and substitute last experiment results
%         try
%             [ms,mv,Trace_tmp] = bayesoptGPML(fun,opt,N0);
%             if withSurrogate
%                 ms_prev=ms;
%                 mv_prev=mv;
%                 Trace_tmp_prev=Trace_tmp;
%             end
% 
%         catch
%             if withSurrogate
%                 ms=ms_prev;
%                 mv=mv_prev;
%                 Trace_tmp=Trace_tmp_prev;
%             else
%                 RAND_ltn = sort(lhsdesign(N_ltn,1));
%                 RAND_ltn_all(:,expr+1)=RAND_ltn;
%                 save(append(dir,'RAND_ltn_all.mat'),'RAND_ltn_all')
%                 Kp_ltn = (Kp_max-Kp_min).*RAND_ltn + Kp_min;
%                 Ki_ltn = (Ki_max-Ki_min).*RAND_ltn + Ki_min;
%                 J_ltn = zeros(N_ltn,1);
%                 for i=1:N_ltn
%                     J_ltn(i) = ObjFun([Kp_ltn(i), Ki_ltn(i)], G);
%                 end
%                 continue
%             end
% 
%         end

        % remove previos data of older surrogate(G2) model, but keep them
        % seperately for plots
        if withSurrogate==true && N~=1 && idx==0
            G2_samples=[G2_samples; Trace_tmp.samples(end-N_G-1,:)];
            G2_values=[G2_values; Trace_tmp.values(end-N_G-1,:)];
            G2_post_mus=[G2_post_mus; Trace_tmp.post_mus(end-N_G-1,:)];
            G2_post_sigma2s=[G2_post_sigma2s; Trace_tmp.post_sigma2s(end-N_G-1,:)];
            idx_G2= [idx_G2;size(Trace_tmp.samples,1)-N_G-1];
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
%     Trace_tmp_removed_G2=Trace_tmp;
%     Trace_tmp_removed_G2.samples(idx_G2,:)=[];
%     Trace_tmp_removed_G2.values(idx_G2)=[];
%     Trace_tmp_removed_G2.post_mus(idx_G2)=[];
%     Trace_tmp_removed_G2.post_sigma2s(idx_G2)=[];
%     Trace_tmp_removed_G2.times(idx_G2)=[];
%     Trace_removed_G2(expr)=Trace_tmp_removed_G2;
%     delete Trace_tmp Trace_tmp_removed_G2

    if withSurrogate==true
        save(append(dir, 'trace_file.mat'),'Trace')
%         save(append(dir, 'trace_file_removed_G2.mat'),'Trace_removed_G2')
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
expr=expr+1;
end

%% Print results
fprintf('******************************************************\n');
fprintf('Best controller gains:      Kp=%2.4f, Ki=%2.4f\n',ms(1),ms(2));
fprintf('Associated cost: J([Kp,Ki])=%2.4f\n',mv);
fprintf('******************************************************\n');

%% Draw optimium
hold on;
plot3([ms(1) ms(1)],[ms(2) ms(2)],[max(j_pt(:)) min(j_pt(:))],'r-','LineWidth',2);
plot3([kp_pt(I) kp_pt(I)],[ki_pt(I) ki_pt(I)],[max(j_pt(:)) min(j_pt(:))],'g-','LineWidth',2);

if withSurrogate
    figName=append(dir, idName,'_SurfGrid_GBO_Solution.png');
else
    figName=append(dir, idName,'_SurfGrid_BO_Solution.png');
end
saveas(gcf,figName)

% %% plots
% if withSurrogate
%     %     experiment=1;
%     %     GBO_plots_one_experiment(ms, Trace, experiment, opt.mins, opt.maxes, N0, N_iter-N_extra, N_G, idName, G)
%     GBO_plots_all_experiments(Trace, N0, N_iter-N_extra, idName)
% end
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
% w=[91.35, 0.34, 0.028, 0.0019];
% w=[40.	0.10	0.01	0.0002];

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
elseif idx==N_G && N_G2_activated_counter<N_G2_activated
    N = N+1;

%     G2_tmp=n4sid(G2data,npG2);
%     G2idtf=idtf(G2_tmp);    [a,b]=tfdata(G2idtf);
%     G2=tf(a,b);
    G2=tfest(G2data, npG2);
    t=0:1/100:3.3;
    y = step(G,t);
    y2 = step(G2,t);

% %     uncomment to check simulation
    figure(1)
    step(G); hold on; step(G2,'r')
    rmse2=sqrt(mean((y-y2).^2))
%     close
    figure(2);
    compare(G2data, G2)
%     close

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
    ytmp=step(CL,eps:sampleTs:sampleTf);
    utmp=step(CLU,eps:sampleTs:sampleTf);
    G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
%     first condition to delete the last simulation after being used
    if N_G2_activated_counter==N_G2_activated && idx==5
        N_G2_activated_counter=N_G2_activated_counter+1;
        idx=0;
    else
        idx= idx +1;
    end
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