function [minsample,minvalue,botrace] = bayesoptGPML_v5(Obj,opt, N0, isGBO)
% ms - best parameter setting found
% mv - best function value for that setting L(ms)
% Trace  - Trace of all settings tried, their function values, and constraint values.
warning('off')
% Check options for minimum level of validity
check_opts(opt);

% Are we doing CBO?
if isfield(opt,'do_cbo') && opt.do_cbo,
    DO_CBO = true;
else
    DO_CBO = false;
    con_values = []; % Dummy value, won't actually be used
end

if isfield(opt,'optimize_ei') && opt.optimize_ei,
    OPT_EI = true;
else
    OPT_EI = false;
end

if isfield(opt,'ei_burnin'),
    EI_BURN = opt.ei_burnin;
else
    EI_BURN = 5;
end

if isfield(opt,'paecordallel_jobs'),
    PAR_JOBS = opt.parallel_jobs;
else
    PAR_JOBS = 1;
end

if isfield(opt,'parallel_mc_iters'),
    MC_ITERS = opt.parallel_mc_iters;
else
    MC_ITERS = 5;
end

% Draw initial candidate grid from a Sobol sequence
if isfield(opt,'grid')
    hyper_grid = scale_point(opt.grid,opt.mins,opt.maxes);
    opt.grid_size = size(hyper_grid,1);
else
    sobol = sobolset(opt.dims);
    hyper_grid = sobol(1:opt.grid_size,:); %creates random values from sobolset in [0,1]
    hyper_grid_record = hyper_grid;%sobol(1:200,:);
    if isfield(opt,'filter_func'), % If the user wants to filter out some candidates
        hyper_grid = scale_point(opt.filter_func(unscale_point(hyper_grid,opt.mins,opt.maxes)),opt.mins,opt.maxes);
    end
end
incomplete = logical(ones(size(hyper_grid,1),1));

% Check for existing trace
if isfield(opt,'resume_trace') && opt.resume_trace && (exist(opt.trace_file,'file') || isfield(opt, 'resume_trace_data'))
    % 		load(opt.trace_file);
    botrace=opt.resume_trace_data;
    %         so initial data needs to be scaled between [0,1] to be in consistent with the code. At the end the unscaled samples are collected as the trace nad final resuts.
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
    if DO_CBO
        con_values = botrace.con_values;
    end
    clear botrace
else
    samples = [];
    values = [];
    con_values = [];
    times = [];
    if isfield(opt,'initial_points')
        %samples = opt.initial_points;
        for i = 1:size(opt.initial_points,1)
            %fprintf('Running initial point #%d...\n',i);
            init_pt = opt.initial_points(i,:);
            sinit_pt = scale_point(init_pt,opt.mins,opt.maxes);
            if ~DO_CBO,
                vali = Obj(init_pt);
            else
                [vali,coni] = Obj(init_pt);
            end
            samples = [samples;sinit_pt];
            values = [values;vali];
            if DO_CBO,
                con_values = [con_values;coni];
            end
        end
    end
    init = floor(rand(1,2)*opt.grid_size);
    %fprintf('Running first point...\n');
    % Get values for the first two samples (no point using a GP yet)
    pt1 = unscale_point(hyper_grid(init(1),:),opt.mins,opt.maxes);
    if ~DO_CBO,
        val1 = Obj(pt1); % First sample
    else
        [val1,con1] = Obj(pt1);
    end

    %fprintf('Running second point...\n');
    pt2 = unscale_point(hyper_grid(init(2),:),opt.mins,opt.maxes);
    if ~DO_CBO,
        val2 = Obj(pt2); % Second sample
    else
        [val2,con2] = Obj(pt2);
    end

    incomplete(init) = false;
    samples = [samples;hyper_grid(init,:)];
    values = [values;val1;val2];
    if DO_CBO,
        con_values = [con_values;con1;con2];
    end

    % Remove first two samples from grid
    hyper_grid = hyper_grid(incomplete,:);
    incomplete = logical(ones(size(hyper_grid,1),1));
end
% Main BO loop
AQ_vals=[];
i_start = length(values) - 2 + 1;
i=i_start;
N_G2=0;
surrogate=false;
G2_trigger_counter=0;
post_mus_record=[];
post_sigma2s_record=[];
botrace.hyper_grid_record=unscale_point(hyper_grid_record,opt.mins,opt.maxes);
while i <opt.max_iters-2+1,
    hidx = -1;
    % samples and hyper_grid should be scaled to [0,1] but hyper_cand is unscaled already
    [hyper_cand,hidx,aq_val, post_mu, post_sigma2] = get_next_cand(samples,values, hyper_grid, opt ,DO_CBO,con_values,OPT_EI,EI_BURN, N0);
    AQ_vals=[AQ_vals;aq_val];

    % Evaluate the candidate with the highest EI to get the actual function value, and add this function value and the candidate to our set.
    tic;
    if isGBO
        eta1=5;
        eta2=0.2;
        %                 fprintf('post_sigma2(hidx)= %d \n', post_sigma2(hidx));
        %                 fprintf('aq_val/max(AQ_vals)= %d \n', aq_val/max(AQ_vals));
        if surrogate==false && post_sigma2(hidx)>eta1 && G2_trigger_counter<20
            G2_trigger_counter=G2_trigger_counter+1;
            fprintf('G2_trigger_counter=%d \n',G2_trigger_counter);
            %                 if aq_val>max(AQ_vals)*eta
            surrogate=true; %switch to use surrogate G2 for objective
            opt.max_iters=opt.max_iters+1;
            counter_deadG2_trial=0; %to switch if for consecutive iterations on surrogate G2 we do not satisfy the improvement condition
        elseif surrogate==true
            if aq_val>max(AQ_vals)*eta2 && N_G2_tmp<30
                opt.max_iters=opt.max_iters+1;
                counter_deadG2_trial=0;
            elseif counter_deadG2_trial<2 && N_G2_tmp<30 %stop if two consecutive poor improvement
                counter_deadG2_trial =counter_deadG2_trial+1;
                opt.max_iters=opt.max_iters+1;
            else
                surrogate=false; %switch back to real plant
                %get optimum gains of BO with surrogate
                [~,idx_tmp]=min(values(end-N_G2_tmp+1:end));
                sample_tmp=samples(end-N_G2_tmp+1:end,:);
                sample_tmp=sample_tmp(idx_tmp,:);
                %masure performance at the optimum gains of BO with surrogate
                [value_tmp,~] = Obj(sample_tmp, surrogate);
                % remove surrogate effect so far and add last data
                times = [times(1:end-N_G2_tmp),0];
                samples = [samples(1:end-N_G2_tmp,:);sample_tmp];
                values = [values(1:end-N_G2_tmp);value_tmp];
                post_mus = [post_mus(1:end-N_G2_tmp);0];
                post_sigma2s=[post_sigma2s(1:end-N_G2_tmp);0];
                AQ_vals=AQ_vals(1:end-N_G2_tmp);
                % Remove closest point on the grid to this candidate from the grid (I use the incomplete vector like this because I will use this vector for other purposes in the future.)
                [~,hidx_tmp]=min(vecnorm(hyper_grid-sample_tmp,2,2));
                incomplete(hidx_tmp) = false;
                hyper_grid = hyper_grid(incomplete,:);
                incomplete = logical(ones(size(hyper_grid,1),1));

                botrace.post_mus=post_mus;
                botrace.post_sigma2s=post_sigma2s;
                botrace.samples = unscale_point(samples,opt.mins,opt.maxes);
                botrace.values = values;
                botrace.times = times;
                if DO_CBO,
                    botrace.con_values = con_values;
                end
                if opt.save_trace
                    save(opt.trace_file,'botrace');
                end
                i=i+1;
                %TODO pay attention may refine code to remove continue
                continue
            end
        end
        [value,N_G2_tmp] = Obj(hyper_cand, surrogate);
    else
        [value] = Obj(hyper_cand);
    end
    times(end+1) = toc;
    samples = [samples;scale_point(hyper_cand,opt.mins,opt.maxes)];
    values(end+1,1) = value;
    post_mus(end+1,1) = post_mu(hidx); %keep the posterior mean where EI is maximum
    post_sigma2s(end+1,1)=post_sigma2(hidx);

    [post_mu_record,post_sigma2_record,~] = get_posterior(samples(1:end-1,:),values(1:end-1),hyper_grid_record,opt,N0);
    post_mus_record = post_mu_record; %keep the posterior mean where EI is maximum
    post_sigma2s_record=post_sigma2_record;
%     post_mus_record=post_mu;
%     post_sigma2s_record=post_sigma2;



    % Remove this candidate from the grid (I use the incomplete vector like this because I will use this vector for other purposes in the future.)
    if hidx >= 0,
        incomplete(hidx) = false;
        hyper_grid = hyper_grid(incomplete,:);
        incomplete = logical(ones(size(hyper_grid,1),1));
    end

    if PAR_JOBS <= 1,
        if ~DO_CBO,
            %fprintf(', value = %f, overall min = %f\n',value,min(values));
        else
            which_feas = all(bsxfun(@le,con_values,opt.lt_const),2);
            %fprintf(', value = %f, feasible = %d, overall min = %f\n',value,all(con_value<=opt.lt_const),min(values(which_feas)));
        end
    else
        if ~DO_CBO,
            %fprintf('Overall min = %f\n\n',min(values));
        else
            which_feas = all(bsxfun(@le,con_values,opt.lt_const),2);
            %fprintf('Overall min = %f\n\n',min(values(which_feas)));
        end
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
    if DO_CBO,
        botrace.con_values = con_values;
    end
    if opt.save_trace
        save(opt.trace_file,'botrace');
    end
    i=i+1;
end

% Get minvalue and minsample
if DO_CBO,
    which_feas = all(bsxfun(@le,con_values,opt.lt_const),2);
    [mv,mi] = min(values(which_feas));
    minvalue = mv;
    fsamples = samples(which_feas,:);
    minsample = unscale_point(fsamples(mi,:),opt.mins,opt.maxes);
else
    [mv,mi] = min(values);
    minvalue = mv;
    minsample = unscale_point(samples(mi,:),opt.mins,opt.maxes);
end

function [hyper_cand,hidx,aq_val, mu, sigma2] = get_next_cand(samples,values,hyper_grid,opt, DO_CBO,con_values,OPT_EI,EI_BURN, N0)
% Get posterior means and variances for all points on the grid.
[mu,sigma2,ei_hyp] = get_posterior(samples,values,hyper_grid,opt,N0);

% Compute EI for all points in the grid, and find the maximum.
if ~DO_CBO,
    best = min(values);
else
    which_feas = all(bsxfun(@le,con_values,opt.lt_const),2);
    best = min(values(which_feas));
    if isempty(best),
        best = max(values)+999;
    end
end
%         given posterior mu and sigma on grid_set points, compute EI taken
%         best sample (at the sampled point(among N0 data or taken by acquisition function) with minimum value(cost))
ei = compute_ei(best,mu,sigma2);
% ei = compute_UCB(mu,sigma2);

hyps = {};
ys = {};
hyps{1} = ei_hyp;
ys{1} = values;

if DO_CBO,
    prFeas = ones(length(ei),1);
    for k = 1:length(opt.lt_const),
        [mu_con,sigma2_con,con_hyp] = get_posterior(samples,con_values(:,k),hyper_grid,opt,N0);
        prFeas = prFeas.*normcdf(repmat(opt.lt_const(k),length(mu_con),1),mu_con,sqrt(sigma2_con));
        hyps{k+1} = con_hyp;
        ys{k+1} = con_values(:,k);
    end
    ei = prFeas.*ei;
end

if OPT_EI && length(values)>EI_BURN,
    hg_star = zeros(size(hyper_grid));
    if ~DO_CBO,
        parfor k = 1:length(hyper_grid),
            z = hyper_grid(k,:);
            zstar = optimize_ei(z,samples,values,best,hyps{1},opt);
            hg_star(k,:) = max(zstar,0);
        end
    else
        parfor k = 1:length(hyper_grid),
            z = hyper_grid(k,:);
            zstar = optimize_eic(z,samples,ys,best,hyps,opt);
            hg_star(k,:) = max(zstar,0);
        end
    end

    [mu,sigma2,ei_hyp] = get_posterior(samples,values,hg_star,opt,N0);
    ei = compute_ei(best,mu,sigma2);
    if DO_CBO,
        prFeas = ones(length(ei),1);
        for k = 1:length(opt.lt_const),
            [mu_con,sigma2_con,con_hyp] = get_posterior(samples,con_values(:,k),hg_star,opt,N0);
            prFeas = prFeas.*normcdf(repmat(opt.lt_const(k),length(mu_con),1),mu_con,sqrt(sigma2_con));
        end
        ei = prFeas.*ei;
    end

    [mei,meidx] = max(ei);
    hyper_cand = unscale_point(hg_star(meidx,:),opt.mins,opt.maxes);
    hidx = -1;
else
    %             find where we have the maximum EI on our grid_set enquiry
    %             data on posterior
    [mei,meidx] = max(ei);
    %             get location(x) on the GP posterior with maximum EI
    hyper_cand = unscale_point(hyper_grid(meidx,:),opt.mins,opt.maxes);
    hidx = meidx;
end
% 		maximum EI acquired at hyper_cand
aq_val = mei;

function [mu,sigma2,hyp] = get_posterior(X,y,x_hats,opt,N0)
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

% calculate hyp: the optimum hyperparameters of GP posterior
% if size(X,1)==N0
hyp = [];
hyp.mean = zeros(n_mh,1);
hyp.cov = zeros(n_ch,1);
hyp.lik = log(0.1);
hyp = minimize(hyp,@gp,-100,@infExact,meanfunc,covfunc,@likGauss,X,y);
% else
%     hyp = [];
%     hyp.mean = zeros(n_mh,1);
%     hyp.cov = zeros(n_ch,1);
%     hyp.lik = log(0.1);
%     hyp = minimize(hyp,@gp,-100,@infExact,meanfunc,covfunc,@likGauss,X(1:end-1,:),y(1:end-1,:));
%     hyp = minimize(hyp,@gp,-100,@infExact,meanfunc,covfunc,@likGauss,X,y);
% end
%     x_hats are test inputs given to gp to predict
% (gp function provides mus and sigma2s of the fitted GP to X, y data evaluated at x_hats)
[mu,sigma2] = gp(hyp,@infExact,meanfunc,covfunc,@likGauss,X,reshape(y,size(X,1),1),x_hats);

function zstar = optimize_ei(z,X,y,best,hyp,opt)

if isfield(opt,'cov_grad_f'),
    cov_grad_f = opt.cov_grad_f;
else
    cov_grad_f = @covSEard_grad;
end
covfunc = opt.covfunc{:};
meanfunc = opt.meanfunc{:};

K = covfunc(hyp.cov,X);
Q = K + exp(2*hyp.lik)*eye(size(K));
prior_mean = meanfunc(hyp.mean,X);
k_hat = covfunc(hyp.cov,X,z);


Qiy = linsolve(Q,y - prior_mean);
Qik = linsolve(Q,k_hat);

zstar = minimize(z,@EI_F,-50,X,Qiy,Qik,cov_grad_f,best,hyp,opt);

function zstar = optimize_eic(z,X,ys,best,hyps,opt)
if isfield(opt,'cov_grad_f'),
    cov_grad_f = opt.cov_grad_f;
else
    cov_grad_f = @covSEard_grad;
end
covfunc = opt.covfunc{:};
meanfunc = opt.meanfunc{:};

Qiys = {};
Qiks = {};

for j = 1:length(hyps),
    Kj = covfunc(hyps{j}.cov,X);
    Q = Kj + exp(2*hyps{j}.lik)*eye(size(Kj));
    prior_mean = meanfunc(hyps{j}.mean,X);
    k_hat = covfunc(hyps{j}.cov,X,z);
    y = ys{j};

    Qiys{j} = linsolve(Q,y - prior_mean);
    Qiks{j} = linsolve(Q,k_hat);
end

zstar = minimize(z,@EIC_F,-50,X,Qiys,Qiks,cov_grad_f,best,hyps,opt);


function [ei,ei_grad] = EI_F(z,X,Qiy,Qik,cov_grad_f,best,hyp,opt)
if nargout > 1,
    [ei,ei_grad] = ei_obj_grad(z,X,Qiy,Qik,cov_grad_f,best,hyp,opt);
    ei = -ei;
    ei_grad = -ei_grad;
else
    ei = ei_obj_grad(z,X,Qiy,Qik,cov_grad_f,best,hyp,opt);
    ei = -ei;
end

function [eic,eic_grad] = EIC_F(z,X,Qiys,Qiks,cov_grad_f,best,hyps,opt)
if nargout > 1,
    [eic,eic_grad] = eic_obj_grad(z,X,Qiys,Qiks,cov_grad_f,best,hyps,opt);
    eic = -eic;
    eic_grad = -eic_grad;
else
    eic = eic_obj_grad(z,X,Qiys,Qiks,cov_grad_f,best,hyps,opt);
    eic = -eic;
end


% Computes EI(z;best) and dEI(z;best)/dz
function [ei,ei_grad] = ei_obj_grad(z,X,Qiy,Qik,cov_grad_f,best,hyp,opt)
covfunc = opt.covfunc{:};
meanfunc = opt.meanfunc{:};

prior_mean = meanfunc(hyp.mean,z);
k_hat = covfunc(hyp.cov,X,z);

% Compute GP predictive posterior
mu = prior_mean + k_hat'*Qiy;
sigma2 = covfunc(hyp.cov,z,'diag') - k_hat'*Qik;
sigma2 = max(sigma2,1e-10);

sigma = sqrt(sigma2);
u = (best - mu) ./ sigma;
ucdf = normcdf(u);
updf = normpdf(u);
ei = sigma .* (u.*ucdf + updf);

if nargout > 1,
    dk_dx = cov_grad_f(hyp.cov,X,z);
    mu_grad = dk_dx'*Qiy;
    s2_grad = (cov_grad_f(hyp.cov,z,z) - 2*Qik'*dk_dx)';

    dEI_dmu = -ucdf;
    dEI_ds2 = updf ./ (2*sigma);

    ei_grad = dEI_dmu.*mu_grad + dEI_ds2.*s2_grad;
end

function [pf,pf_grad] = pf_obj_grad(z,X,Qiy,Qik,cov_grad_f,lambda,hyp,opt)
covfunc = opt.covfunc{:};
meanfunc = opt.meanfunc{:};

prior_mean = meanfunc(hyp.mean,z);
k_hat = covfunc(hyp.cov,X,z);

mu = prior_mean + k_hat'*Qiy;
sigma2 = covfunc(hyp.cov,z,'diag') - k_hat'*Qik;
sigma = sqrt(sigma2);

pf = normcdf(lambda,mu,sigma);

if nargout > 1,
    dk_dx = cov_grad_f(hyp.cov,X,z);
    mu_grad = dk_dx'*Qiy;
    s2_grad = (cov_grad_f(hyp.cov,z,z) - 2*Qik'*dk_dx)';

    Z = (lambda - mu) ./ sigma;
    pf_grad = -normpdf(Z)*(mu_grad./sigma + s2_grad/(2*sigma2)*Z);
end

function [eic,eic_grad] = eic_obj_grad(z,X,Qiys,Qiks,cov_grad_f,best,hyps,opt)
if nargout > 1,
    [ei,ei_grad] = ei_obj_grad(z,X,Qiys{1},Qiks{1},cov_grad_f,best,hyps{1},opt);
else
    ei = ei_obj_grad(z,X,Qiys{1},Qiks{1},cov_grad_f,best,hyps{1},opt);
end

pfs = zeros(length(opt.lt_const),1);

if nargout > 1,
    pf_grads = zeros(length(opt.lt_const),length(z));
end

for j = 1:length(opt.lt_const),
    Qiy = Qiys{j+1};
    Qik = Qiks{j+1};
    hyp = hyps{j+1};
    lambda = opt.lt_const(j);

    if nargout > 1,
        [pf,pf_grad] = pf_obj_grad(z,X,Qiy,Qik,cov_grad_f,lambda,hyp,opt);
    else
        [pf,~] = pf_obj_grad(z,X,Qiy,Qik,cov_grad_f,lambda,hyp,opt);
    end

    pfs(j) = pf;

    if nargout > 1,
        pf_grads(j,:) = pf_grad;
    end
end

eic = ei .* prod(pfs);

if nargout > 1,
    eic_grad = ei_grad .* prod(pfs);
    for j = 1:length(opt.lt_const),
        gradterm = (ei .* pf_grads(j,:) .* (prod(pfs)/pfs(j)))';
        eic_grad = eic_grad + gradterm;
    end
end


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

% Returns the derivative of covSEard w.r.t. a single candidate z
function [k] = covSEard_grad(hyp,x,z)
D = length(z);
ell = exp(hyp(1:D));
sf2 = exp(2*hyp(D+1));

k = covSEard(hyp,x,z);
sq_der = (diag(-2./(ell.^2))*(bsxfun(@minus,x,z))')';
k = -0.5*bsxfun(@times,k,sq_der);

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


