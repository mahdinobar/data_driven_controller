function [minsample,minvalue,botrace,LVgains, hyper_grid, idx_G2, G2, counter_s,when_switch_s, debug_ratio, ys] = LV_bayesoptGPML_v5_switch2(Obj,opt, hyper_grid,counter_s,G2data,idx_G2,when_switch_s, counter_real)
% version 5: remove surrogate data from BO dataset when you switch back to
% real plant
% ms - best parameter setting found
% mv - best function value for that setting L(ms)
% Trace  - Trace of all settings tried, their function values, and constraint values.
warning('off')
% Check options for minimum level of validity
check_opts(opt);
incomplete = true(size(hyper_grid,1),1);
% Check for existing trace
% try 
    botrace=opt.resume_trace_data;
    %         so initial data needs to be scaled between [0,1] to be in consistent with the code. At the end the unscaled samples are collected as the trace and final resuts.
    samples = scale_point(botrace.samples,opt.mins,opt.maxes);
    values = botrace.values;
    times = botrace.times;
    if ~isfield(botrace, 'post_mus')
        post_mus=zeros(size(botrace.values));
        post_sigma2s=zeros(size(botrace.values));
    else
        post_mus=botrace.post_mus;
        post_sigma2s=botrace.post_sigma2s;
    end
% catch
%     error("Problem in resume_trace!")
% end
% Main BO iteration
if isfield(botrace,'AQ_vals')
    AQ_vals=botrace.AQ_vals;
else
    AQ_vals=[];
end

% samples and hyper_grid should be scaled to [0,1] but hyper_cand is unscaled already
[hyper_cand,hidx,aq_val, post_mu, post_sigma2, ~] = get_next_cand(samples,values, hyper_grid, opt, botrace);
AQ_vals=[AQ_vals;aq_val];
% Evaluate the candidate with the highest EI to get the actual function value, and add this function value and the candidate to our set.
tic;

if ~isempty(when_switch_s)
    N0=1;
    npG2=2;
    nzG2=1;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
            Trace_tmp.samples(idx_G2,:)=[];
        Trace_tmp.values(idx_G2)=[];
        Trace_tmp.post_mus(idx_G2)=[];
        Trace_tmp.post_sigma2s(idx_G2)=[];
        Trace_tmp.times(idx_G2)=[];
        Trace_tmp.AQ_vals(idx_G2)=[];
    ys=[];
    for k=1+N0:length(Trace_tmp.values)
        Kp = Trace_tmp.samples(k,1);
        Ti = 1/Trace_tmp.samples(k,2);
        Td = 0;
        N=inf;
        Ts = 0.01;
        C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
        CL=feedback(C*G2, 1);
        reference0=0;
        reference=40;
        t_high=(11*Ts):Ts:(5.1-Ts);
        t_low=0:Ts:(10*Ts);
        step_high=reference.*ones(length(t_high),1);
        step_low=reference0.*ones(length(t_low),1);
        t=[t_low,t_high]';
        r=[step_low;step_high];
        y2=lsim(CL,r,t);
        y_high=y2(t>(.1)); %TODO check pay attention
        t_high=t(t>(.1));%TODO check
        e=abs(y_high-reference);
        ITAE = trapz(t_high, t_high.*abs(e));
        S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.05);
        st=S.SettlingTime;
        ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
        Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
        if isnan(st) || st>5
            st=5;
        end
        if ITAE>(reference-reference0)*40
            ITAE=(reference-reference0)*40;
        end
        if isnan(Tr) || Tr>5
            Tr=5;
        end
        if isnan(ov) || ov>1
            ov=1;
        end
        perf_Data=[ov, Tr, st, ITAE];
        value = Obj(perf_Data);
        ys=[ys;value];
        save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_6/GBO_2_tmp_1/debug_tmp.mat")
    end
    sigma2_s=sqrt(1/(length(Trace_tmp.values)-N0)*sum(ys-Trace_tmp.values));
else
    sigma2_s=0;
    ys=[0];
end
debug_ratio=post_sigma2(hidx)/sigma2_s;
eta1=1;
eta2=0.2;
if counter_s==0 && post_sigma2(hidx)/sigma2_s>eta1 && length(when_switch_s)<15 %put safety that not switch more than certain times to surrogate
    counter_s=1; %to switch if for consecutive iterations on surrogate G2 we do not satisfy the improvement condition
    when_switch_s=[when_switch_s;counter_real];
elseif counter_s>0
    if aq_val>max(AQ_vals)*eta2 %limit also total number of surrogate data
        counter_s = 1; 
    elseif counter_s<3+1 %means if 3 consecutive iterations with surrogate there is no considerable improvement we stop BO on surrogate
        counter_s =counter_s+1;
    else
        save(append("C:\mahdi\data_driven_controller\Data\exper_72_4\GBO_v5_1\debug_data.mat"))
        counter_s=0; %switch back to real system
        % find min observed with the last surrogate
        [~,I]=min(botrace.values(idx_G2));
        hyper_cand = botrace.samples(idx_G2(I),:);
        % set/keep relevant data
        post_mu=botrace.post_mus(idx_G2(I));
        post_sigma2=botrace.post_sigma2s(idx_G2(I));
        hidx=1;
        aq_val=botrace.AQ_vals(idx_G2(I)-1);
        % remove surrogate data
        botrace.samples(idx_G2,:)=[];
        botrace.values(idx_G2)=[];
        botrace.post_mus(idx_G2)=[];
        botrace.post_sigma2s(idx_G2)=[];
        botrace.times(idx_G2)=[];
        botrace.AQ_vals(idx_G2-1)=[];
        % corrections for code consistency
        AQ_vals=botrace.AQ_vals;
        AQ_vals=[AQ_vals;aq_val];
        samples = scale_point(botrace.samples,opt.mins,opt.maxes);
        values = botrace.values;
        times = botrace.times;
        post_mus=botrace.post_mus;
        post_sigma2s=botrace.post_sigma2s;
        % reset surrogate indices
        idx_G2=[];
    end
end
% Remove this candidate from the grid (I use the incomplete vector like this because I will use this vector for other purposes in the future.)
if hidx >= 0
    incomplete(hidx) = false;
    hyper_grid = hyper_grid(incomplete,:);
    incomplete = true(size(hyper_grid,1),1);
end
LVgains=hyper_cand;
if counter_s==0
    G2=[];
elseif counter_s>0
    npG2=2;
    nzG2=1;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2 = tfest(G2data, npG2,nzG2,Options, 'Ts', 10e-3);
    Kp = hyper_cand(1);
    Ti = 1/hyper_cand(2);
    Td = 0;
    N=inf;
    Ts = 0.01;
    C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
    CL=feedback(C*G2, 1);
    reference0=0;
    reference=40;
    t_high=(11*Ts):Ts:(5.1-Ts);
    t_low=0:Ts:(10*Ts);
    step_high=reference.*ones(length(t_high),1);
    step_low=reference0.*ones(length(t_low),1);
    t=[t_low,t_high]';
    r=[step_low;step_high];
    y2=lsim(CL,r,t);
    y_high=y2(t>.1);
    t_high=t(t>.1);%TODO check
    e=abs(y_high-reference);
    ITAE = trapz(t_high, t_high.*abs(e));
    S = lsiminfo(y_high,t_high,reference,reference0);
    st=S.SettlingTime;
    ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
    Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
    if isnan(st) || st>5
        st=5;
    end
    if ITAE>(reference-reference0)*40
        ITAE=(reference-reference0)*40;
    end
    if isnan(Tr) || Tr>5
        Tr=5;
    end
    if isnan(ov) || ov>1
        ov=1;
    end
    perf_Data=[ov, Tr, st, ITAE];
    value = Obj(perf_Data);
    values(end+1,1) = value;
    idx_G2=[idx_G2;size(values,1)];
    samples = [samples;scale_point(hyper_cand,opt.mins,opt.maxes)];
    botrace.samples = unscale_point(samples,opt.mins,opt.maxes);
end
times(end+1) = toc;
post_mus(end+1,1) = post_mu(hidx); %keep the posterior mean where EI is maximum
post_sigma2s(end+1,1)=post_sigma2(hidx);
botrace.post_mus=post_mus;
botrace.post_sigma2s=post_sigma2s;
botrace.values = values;
botrace.times = times;
botrace.AQ_vals=AQ_vals;
% Get minvalue and minsample
[mv,mi] = min(values);
minvalue = mv;
minsample = unscale_point(samples(mi,:),opt.mins,opt.maxes);

function [hyper_cand,hidx,aq_val, mu, sigma2, ei_hyp] = get_next_cand(samples,values,hyper_grid,opt, botrace)
% Get posterior means and variances for all points on the grid.
[mu,sigma2,ei_hyp] = get_posterior(samples,values,hyper_grid,opt,botrace);
% Compute EI for all points in the grid, and find the maximum.
best = min(values);
%         given posterior mu and sigma on grid_set points, compute EI taken
%         best sample (at the sampled point(among N0 data or taken by acquisition function) with minimum value(cost))
ei = compute_ei(best,mu,sigma2);
% ei = compute_UCB(mu,sigma2);
%             find where we have the maximum EI on our grid_set enquiry
%             data on posterior
[mei,meidx] = max(ei);
%             get location(x) on the GP posterior with maximum EI
hyper_cand = unscale_point(hyper_grid(meidx,:),opt.mins,opt.maxes);
hidx = meidx;
% 		maximum EI acquired at hyper_cand
aq_val = mei;

function [mu,sigma2,hyp,post] = get_posterior(X,y,x_hats,opt,botrace)
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

if ~isfield(botrace, 'hyp_GP_mean')
    botrace.hyp_GP_mean=zeros(n_mh,1);
    botrace.hyp_GP_cov=zeros(n_ch,1)';
    botrace.hyp_GP_lik=log(0.1);
end
% stop optimizing prior hyperparameters after certain iterations
if length(botrace.hyp_GP_mean)<50
    %     calculate hyp: the optimum hyperparameters of prior model
    hyp = [];
    hyp.mean = zeros(n_mh,1);
    hyp.cov = zeros(n_ch,1);
    hyp.lik = log(0.1); %log(noise standard deviation)
    hyp = minimize(hyp,@gp,-100,@infExact,meanfunc,covfunc,@likGauss,X,y); %here we optimize negative log marginal likelihood given training data so far with respect to the mean and kernel hyperparameters to find optimum hyperparameters
else
    hyp.mean = botrace.hyp_GP_mean(end, :)';
    hyp.cov = botrace.hyp_GP_cov(end,:)';
    hyp.lik = botrace.hyp_GP_lik(end);
end

%     x_hats are test inputs given to gp to predict
% (gp function provides mus and sigma2s of the fitted GP to X, y data evaluated at x_hats)
[mu,sigma2,~,~,~,post] = gp(hyp,@infExact,meanfunc,covfunc,@likGauss,X,reshape(y,size(X,1),1),x_hats);

function ei = compute_ei(best,mu,sigma2)
sigmas = sqrt(sigma2);
beta=0; % increasing beta increases the exploration of EI
u = (best + beta - mu) ./ sigmas;
ucdf = normcdf(u);
updf = normpdf(u);
ei = sigmas .* (u .* ucdf + updf);

function UCB = compute_UCB(mu,sigma2)
sigmas = sqrt(sigma2);
beta=10; % increasing beta increases the exploration of UCB
UCB = mu + beta.*sigmas;

function upt = unscale_point(x,mins,maxes)
if size(x,1) == 1
    upt = x .* (maxes - mins) + mins;
else
    upt = bsxfun(@plus,bsxfun(@times,x,(maxes-mins)),mins);
end

function pt = scale_point(x,mins,maxes)
pt = bsxfun(@rdivide,bsxfun(@minus,x,mins),maxes-mins);

function check_opts(opt)
if ~isfield(opt,'dims')
    error('bayesopt:opterror',['The dims option specifying the dimensionality of ' ...
        'the optimization problem is required']);
end
if ~isfield(opt,'mins') || length(opt.mins) < opt.dims
    error('bayesopt:opterror','Must specify minimum values for each hyperparameter');
end
if ~isfield(opt,'maxes') || length(opt.maxes) < opt.dims
    error('bayesopt:opterror','Must specify maximum values for each hyperparameter');
end
if isfield(opt,'parallel_jobs') && isfield(opt,'optimize_ei') && opt.parallel_jobs > 1,
    error('bayesopt:opterror','Parallel jobs with optimize_ei on is not supported yet.');
end
if isfield(opt,'optimize_ei') && opt.optimize_ei && ~isfield(opt,'cov_grad_f'),
    warning('bayesopt:optwarning','Warning: opt.optimize_ei is set, but opt.cov_grad_f is not. By default, covSEard is assumed.');
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

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% test_dir="C:\mahdi\data_driven_controller\Data\TEST000000000000000000000000\";
% mkdir(test_dir)
% save(test_dir,'opt.resume_trace_data')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%