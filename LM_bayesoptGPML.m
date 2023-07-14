function [minsample,minvalue,botrace] = LM_bayesoptGPML(Obj,opt, N0)
% ms - best parameter setting found
% mv - best function value for that setting L(ms)
% Trace  - Trace of all settings tried, their function values, and constraint values.
%% Check options for minimum level of validity
check_opts(opt);
%% Draw initial candidate grid from a Sobol sequence
if isfield(opt,'grid')
    hyper_grid = scale_point(opt.grid,opt.mins,opt.maxes);
    opt.grid_size = size(hyper_grid,1);
    hyper_grid_record = hyper_grid;
else
    sobol = sobolset(opt.dims);
    hyper_grid = sobol(1:opt.grid_size,:); %creates random values from sobolset in [0,1]
    hyper_grid_record = hyper_grid;%sobol(1:200,:);
    if isfield(opt,'filter_func'), % If the user wants to filter out some candidates
        hyper_grid = scale_point(opt.filter_func(unscale_point(hyper_grid,opt.mins,opt.maxes)),opt.mins,opt.maxes);
    end
end
incomplete = logical(ones(size(hyper_grid,1),1));
%% Check for existing trace
if isfield(opt,'resume_trace') && opt.resume_trace && (exist(opt.trace_file,'file') || isfield(opt, 'resume_trace_data'))
    % 		load(opt.trace_file);
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
    clear botrace
else
    samples = [];
    values = [];
    times = [];
    if isfield(opt,'initial_points')
        %samples = opt.initial_points;
        for i = 1:size(opt.initial_points,1)
            %fprintf('Running initial point #%d...\n',i);
            init_pt = opt.initial_points(i,:);
            sinit_pt = scale_point(init_pt,opt.mins,opt.maxes);
            vali = Obj(init_pt);
            samples = [samples;sinit_pt];
            values = [values;vali];
        end
    end
    init = floor(rand(1,2)*opt.grid_size);
    % Get values for the first two samples (no point using a GP yet)
    pt1 = unscale_point(hyper_grid(init(1),:),opt.mins,opt.maxes);
    val1 = Obj(pt1); % First sample
    pt2 = unscale_point(hyper_grid(init(2),:),opt.mins,opt.maxes);
    val2 = Obj(pt2); % Second sample
    incomplete(init) = false;
    samples = [samples;hyper_grid(init,:)];
    values = [values;val1;val2];
    % Remove first two samples from grid
    hyper_grid = hyper_grid(incomplete,:);
    incomplete = logical(ones(size(hyper_grid,1),1));
end
%% Main BO loop
AQ_vals=[];
i_start = length(values) - 2 + 1;
i=i_start;
N_G2=0;
surrogate=false;
G2_trigger_counter=0;
post_mus_record=[];
post_sigma2s_record=[];
botrace.hyper_grid_record=unscale_point(hyper_grid_record,opt.mins,opt.maxes);
botrace.idx_G2_samples=[];
hyp_GP_mean=[];
hyp_GP_cov=[];
hyp_GP_lik=[];
GP_hypers_mean_record=[];
GP_hypers_cov_record=[];
GP_hypers_lik_record=[];
total_G2_after_activation=0;
removed_points=[];
idx_G2_samples=[];
i_tmp=0;
real_counter=0;
real_counter_limit=5;
limit_total_G2_after_activation=20;
while i <opt.max_iters-2+1
    hidx = -1;
    % samples and hyper_grid should be scaled to [0,1] but hyper_cand is unscaled already
    [hyper_cand,hidx,aq_val, post_mu, post_sigma2, hyp_GP] = get_next_cand(samples,values, hyper_grid, opt, N0, botrace);
    AQ_vals=[AQ_vals;aq_val];
    % Evaluate the candidate with the highest EI to get the actual function value, and add this function value and the candidate to our set.
    tic;
    eta1=opt.eta1;
    eta2=opt.eta2;
    fprintf('total_G2_after_activation= %d \n', total_G2_after_activation);
    if surrogate==false && post_sigma2(hidx)>eta1 && total_G2_after_activation<limit_total_G2_after_activation+1 && real_counter<real_counter_limit+1%also stop if more than 10 times after last activation used G2
        surrogate=true; %switch to use surrogate G2 for objective
        opt.max_iters=opt.max_iters+1;
        counter=1; %to switch if for consecutive iterations on surrogate G2 we do not satisfy the improvement condition
        total_G2_after_activation=total_G2_after_activation+1;
        % remove older surrogate data from D
        times(idx_G2_samples)=[];
        samples(idx_G2_samples,:)=[];
        values(idx_G2_samples)=[];
        post_mus(idx_G2_samples)=[];
        post_sigma2s(idx_G2_samples)=[];
        AQ_vals(idx_G2_samples-1)=[];
        botrace.idx_G2_samples=idx_G2_samples;
        botrace.times=times;
        botrace.samples=unscale_point(samples,opt.mins,opt.maxes);
        botrace.values=values;
        botrace.post_mus=post_mus;
        botrace.post_sigma2s=post_sigma2s;
        botrace.AQ_vals=AQ_vals;
        i_tmp=i_tmp+length(idx_G2_samples);
        idx_G2_samples=[];
    elseif surrogate==true
        if aq_val>max(AQ_vals)*eta2 && total_G2_after_activation<limit_total_G2_after_activation+1 && real_counter<real_counter_limit+1
            opt.max_iters=opt.max_iters+1;
            counter = 1; %in server GBO_72 and 74results this was missing
            total_G2_after_activation=total_G2_after_activation+1;
        elseif counter<1+1 && total_G2_after_activation<limit_total_G2_after_activation+1 && real_counter<real_counter_limit+1
            counter =counter+1;
            opt.max_iters=opt.max_iters+1;
            total_G2_after_activation=total_G2_after_activation+1;
        else
            surrogate=false;
            % put back removed points from hyper_grid when we used surrogate
            hyper_grid=[hyper_grid;removed_points]; %check if you need to reset removed_points
            total_G2_after_activation=0;
        end
    end
    value = Obj(hyper_cand, surrogate);
    times(end+1) = toc;
    samples = [samples;scale_point(hyper_cand,opt.mins,opt.maxes)];
    values(end+1,1) = value;
    post_mus(end+1,1) = post_mu(hidx); %keep the posterior mean where EI is maximum
    hyp_GP_mean=[hyp_GP_mean; hyp_GP.mean'];
    hyp_GP_cov=[hyp_GP_cov; hyp_GP.cov'];
    hyp_GP_lik=[hyp_GP_lik; hyp_GP.lik'];
    post_sigma2s(end+1,1)=post_sigma2(hidx);
    %     keep last GP model evaluated at records hyper grid
    [post_mu_record,post_sigma2_record,GP_hypers_record,GP_posterior_record] = get_posterior(samples(1:end-1,:),values(1:end-1),hyper_grid_record,opt,N0, botrace);
    post_mus_record=[post_mus_record,post_mu_record];
    post_sigma2s_record=[post_sigma2s_record,post_sigma2_record];
    GP_hypers_mean_record=[GP_hypers_mean_record,GP_hypers_record.mean];
    GP_hypers_cov_record=[GP_hypers_cov_record,GP_hypers_record.cov];
    GP_hypers_lik_record=[GP_hypers_lik_record,GP_hypers_record.lik];
    % Remove this candidate from the grid (I use the incomplete vector like this because I will use this vector for other purposes in the future.)
    if hidx >= 0,
        incomplete(hidx) = false;
        % keep removed point from hyper_grid when we use surrogate to get them back
        if surrogate==true
            removed_points=[removed_points;hyper_grid(~incomplete,:)];
            idx_G2_samples=[idx_G2_samples;i+2-i_tmp]; %todo why i+2? check
        elseif surrogate==false
            real_counter=real_counter+1;
        end
        hyper_grid = hyper_grid(incomplete,:);
        incomplete = logical(ones(size(hyper_grid,1),1));
    end
    botrace.post_mus=post_mus;
    botrace.post_sigma2s=post_sigma2s;
    botrace.post_mus=post_mus;
    botrace.post_sigma2s=post_sigma2s;
    botrace.samples = unscale_point(samples,opt.mins,opt.maxes);
    botrace.values = values;
    botrace.times = times;
    botrace.post_mus_record=post_mus_record;
    botrace.post_sigma2s_record=post_sigma2s_record;
    botrace.GP_hypers_mean_record=GP_hypers_mean_record;
    botrace.GP_hypers_cov_record=GP_hypers_cov_record;
    botrace.GP_hypers_lik_record=GP_hypers_lik_record;
    botrace.hyp_GP_lik=hyp_GP_lik;
    botrace.hyp_GP_cov=hyp_GP_cov;
    botrace.hyp_GP_mean=hyp_GP_mean;
    botrace.GP_posterior_record=GP_posterior_record;
    botrace.AQ_vals=AQ_vals;
    if opt.save_trace_tmp
        save(opt.trace_file_tmp,'botrace');
    end
    i=i+1;
end
% remove surrogate data from D
times(idx_G2_samples)=[];
samples(idx_G2_samples,:)=[];
values(idx_G2_samples)=[];
post_mus(idx_G2_samples)=[];
post_sigma2s(idx_G2_samples)=[];
AQ_vals(idx_G2_samples-1)=[];
botrace.idx_G2_samples=idx_G2_samples;
botrace.times=times;
botrace.samples=unscale_point(samples,opt.mins,opt.maxes);
botrace.values=values;
botrace.post_mus=post_mus;
botrace.post_sigma2s=post_sigma2s;
botrace.AQ_vals=AQ_vals;
% Get minvalue and minsample
[mv,mi] = min(values);
minvalue = mv;
minsample = unscale_point(samples(mi,:),opt.mins,opt.maxes);
%%
function [hyper_cand,hidx,aq_val, mu, sigma2, ei_hyp] = get_next_cand(samples,values,hyper_grid,opt, N0, botrace)
% Get posterior means and variances for all points on the grid.
[mu,sigma2,ei_hyp] = get_posterior(samples,values,hyper_grid,opt,N0, botrace);
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
%%
function [mu,sigma2,hyp,post] = get_posterior(X,y,x_hats,opt,N0, botrace)
meanfunc = opt.meanfunc;
covfunc = opt.covfunc;
n_mh = num_hypers(meanfunc{1},opt);
n_ch = num_hypers(covfunc{1},opt);
% stop optimizing prior hyperparameters after certain iterations
if isfield(botrace,"samples")
    iter_prior_opt=length(botrace.samples);
else
    iter_prior_opt=0;
end
if iter_prior_opt<opt.iter_stop_prior_opt
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
%  x_hats are test inputs given to gp to predict
% (gp function provides mus and sigma2s of the fitted GP to X, y data evaluated at x_hats)
[mu,sigma2,~,~,~,post] = gp(hyp,@infExact,meanfunc,covfunc,@likGauss,X,reshape(y,size(X,1),1),x_hats);
%% EI
function ei = compute_ei(best,mu,sigma2)
sigmas = sqrt(sigma2);
beta=0; % increasing beta increases the exploration of EI
u = (best + beta - mu) ./ sigmas;
ucdf = normcdf(u);
updf = normpdf(u);
ei = sigmas .* (u .* ucdf + updf);
%% UCB
function UCB = compute_UCB(mu,sigma2)
sigmas = sqrt(sigma2);
beta=10; % increasing beta increases the exploration of UCB
UCB = mu + beta.*sigmas;
%% unscale
function upt = unscale_point(x,mins,maxes)
if size(x,1) == 1
    upt = x .* (maxes - mins) + mins;
else
    upt = bsxfun(@plus,bsxfun(@times,x,(maxes-mins)),mins);
end
%% scale
function pt = scale_point(x,mins,maxes)
pt = bsxfun(@rdivide,bsxfun(@minus,x,mins),maxes-mins);
%% checkpoints
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
%%
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
