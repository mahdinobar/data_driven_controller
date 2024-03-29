% Guided BO and BO for Linear Motor System
%% clean, set directories, start OPC server
clear all; clc; close all;
global P_crop_safe D_crop_safe exp_data_crop_safe sampleTs N idx G2data N_G2 debugging metric_error_all metric_gt_all metric_hat_all
metric_error_all=[];
metric_gt_all=[];
metric_hat_all=[];
%% user manual iputs
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data';
idName= 'LM_203_debug';
sys='LM';
isGBO=true;
N0=1; %TODO if N0~=1
N_batch=1;
N_iter=30;
eta1=7.5827e-08;
eta2=0.2;
debugging=true;
sampleTs=0.001;
%% misc settings and N0 settings

if isGBO==true
    dir=append(tmp_dir,'/', idName, '/GBO/');
    load(append(tmp_dir,'/', idName, '/N0_RAND_all.mat'))
    if not(isfolder(dir))
        mkdir(dir)
    end
else
    eta1=inf;
    dir=append(tmp_dir,'/', idName, '/BO/');
    if not(isfolder(dir))
        mkdir(dir)
    end
    N0_RAND_all=zeros(N0,N_batch);
    for batch=1:N_batch
        RAND = sort(lhsdesign(N0,1));
        N0_RAND_all(:,batch)=RAND;
    end
    save(append(tmp_dir,'/', idName, '/N0_RAND_all.mat'),'N0_RAND_all')
end
% set seed of all random generations
rng(1,'twister');
%% loads
% load gain limits (feasible set)
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_KpKd_bounds.mat')
% load and prepare LM offline dataset
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data.mat")

idx_crop_safe=logical((P_safe<Kp_max).*(P_safe>Kp_min).*(D_safe<Kd_max).*(D_safe>Kd_min));
P_crop_safe=P_safe(idx_crop_safe);
D_crop_safe=D_safe(idx_crop_safe);
exp_data_crop_safe=exp_data_safe;
exp_data_crop_safe.actPos_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actCur_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actVel_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.P(~idx_crop_safe)=[];
exp_data_crop_safe.D(~idx_crop_safe)=[];

%% Setup the Gaussian Process (GP) Library
addpath ./gpml/
startup;
opt.meanfunc={@meanConst};
opt.covfunc={@covMaternard, 5};
opt.dims = 2; % Number of parameters.
opt.mins = [Kp_min, Kd_min]; % Minimum value for each of the parameters. Should be 1-by-opt.dims
opt.maxes = [Kp_max, Kd_max]; % Vector of maximum values for each parameter.
% opt.grid_size = 20000;
opt.grid=[P_crop_safe,D_crop_safe]; %use grid for offline LM dataset
opt.trace_file=append(dir,'trace_file.mat');
opt.trace_file_tmp=append(dir,'trace_file_tmp.mat');
opt.resume_trace=true;
% save temporarily until here the trace file: comment for faster
opt.save_trace_tmp=0;
% set maximum iteration to stop optimizing prior hyperparameters(set inf to disable)
opt.iter_stop_prior_opt=inf;
opt.eta1=eta1;
opt.eta2=eta2;
%% We define the function we would like to optimize
fun = @(X, surrogate)ObjFun_Guided(X, surrogate);
%% Start the optimization
step_high=40;
step_down=30;
% each experiment is the entire iterations starting with certain initial set
for batch=1:1:N_batch
    fprintf('############################ BATCH: %d \n', batch);
    N=0;
    idx=[];
    N_G2=0;
    G2_samples=[];
    G2_values=[];
    G2_post_mus=[];
    G2_post_sigma2s=[];
    % create initial dataset per experiment
    RAND=N0_RAND_all(:,batch);
    range_kp=Kp_max-Kp_min;
    range_kd=Kd_max-Kd_min;
    Kp_ltn = (Kp_max-Kp_min).*RAND + Kp_min;
    Kd_ltn = (Kd_max-Kd_min).*RAND + Kd_min;
    [~,I_tmp]=min((P_crop_safe-Kp_ltn).^2+(D_crop_safe-Kd_ltn).^2);
    Kp_ltn=P_crop_safe(I_tmp);
    Kd_ltn=D_crop_safe(I_tmp);
    J_ltn = zeros(N0,1);
    for i=1:N0
        exp_data=LinMotor(Kp_ltn(i), Kd_ltn(i));
        J_ltn(i) = ObjFun(exp_data,[],[]);
        sample_idx=exp_data.r==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_offset=exp_data.actPos(tmp_idx(1)-10);
        u_offset=exp_data.actCur(tmp_idx(1)-10);
        % use 50 ms of data after step high for G2
        ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
        utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
        vtmp = diff(ytmp((50+3):end))./sampleTs;
        utmp = utmp((50+3):(end-1));
        if i==1
            G2data = iddata(vtmp,utmp,sampleTs);
        else
            G2data = merge(G2data, iddata(vtmp,utmp,sampleTs));
        end
    end
    % set initial dataset
    X_ltn=[Kp_ltn, Kd_ltn];
    y_ltn=J_ltn;
    botrace0.samples=X_ltn;
    botrace0.values=y_ltn;
    botrace0.idx_G2_samples=[];
    botrace0.times=RAND'; %todo
    opt.resume_trace_data = botrace0;
    clear botrace0
    opt.max_iters = size(opt.resume_trace_data.samples,1)+N_iter;
    [ms,mv,Trace_tmp] = LM_bayesoptGPML(fun,opt,N0);
    Trace(batch)=Trace_tmp;
    clearvars Trace_tmp
    save(append(dir, 'trace_file.mat'),'Trace')
end

%%
function [objective] = ObjFun(exp_data, G2, gains)
global sampleTs
global debugging
reference0=30;
reference=40;
if isempty(G2)==1
    step_high=40;
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=0;%exp_data.actPos(tmp_idx(1)-10);
    u_offset=0;%exp_data.actCur(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    y_high=ytmp(50:end); %todo check
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=mean(exp_data.actPos((tmp_idx(1)-60):(tmp_idx(1)-10)))-y_offset;
    y_final=mean(exp_data.actPos((tmp_idx(end)-60):(tmp_idx(end)-10)))-y_offset;
    % manually calculate settling time for server because server lsiminfo is wrong
    i_st = max(find(abs(y_high-y_final)>0.02*(y_final-y_init)));
    st=t_high(i_st+1);
    if isnan(st)
        st=3;
    end
    if max(y_high)>reference
        ov=max(0,(max(y_high)-y_init)/(y_final-y_init)-1);
    else
        ov=0;
    end
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=y_high-reference;
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), abs(e(1:ceil(5*Tr*1000))));
    e_ss=abs(y_final-reference);
elseif isempty(G2)==0 %when we use surrogate to estimate objective
    F=0.001;
    Kp=gains(1)/512;
    Kd=gains(2)/768;
    Ki=0;
    %     G2c=d2c(G2);
    %     G2_num=G2c.Numerator{1};
    %     G2_den=G2c.Denominator{1};
    [A,B,C,D]=tf2ss(cell2mat(G2.Numerator),cell2mat(G2.Denominator));
    try
        mdlWks = get_param('DT','ModelWorkspace');
    catch
        DT
        mdlWks = get_param('DT','ModelWorkspace');
    end
    assignin(mdlWks,'sampleTs',sampleTs)
    assignin(mdlWks,'Kp',Kp)
    assignin(mdlWks,'Kd',Kd)
    assignin(mdlWks,'Ki',Ki)
    assignin(mdlWks,'F',F)
    assignin(mdlWks,'reference0',reference0)
    assignin(mdlWks,'reference',reference)
    %         assignin(mdlWks,'G2_den',G2_den)
    %         assignin(mdlWks,'G2_num',G2_num)
    assignin(mdlWks,'A',A)
    assignin(mdlWks,'B',B)
    assignin(mdlWks,'C',C)
    assignin(mdlWks,'D',D)
    assignin(mdlWks,'reference0',reference0)
    simOut = sim("DT.slx");
    y2=simOut.yout{1}.Values.Data(1:1:end-1);
    t=simOut.yout{1}.Values.Time(1:1:end-1);
    y_high=y2(t>(50*sampleTs)); %TODO check pay attention
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=0;
    y_final=mean(y_high(end-5:end));
    % manually calculate settling time for server because server lsiminfo is wrong
    i_st = max(find(abs(y_high-y_final)>0.02*(y_final-y_init)));
    st=t_high(i_st+1);
    if isnan(st)
        st=3;
    end
    if max(y_high)>reference
        ov=max(0,(max(y_high)-y_init)/(y_final-y_init)-1);
    else
        ov=0;
    end
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=y_high-reference;
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), abs(e(1:ceil(5*Tr*1000))));
    e_ss=abs(y_final-reference);
    if debugging==true
        global metric_error_all metric_gt_all metric_hat_all
        %% debug
        exp_data=LinMotor(gains(1),gains(2));
        step_high=40;
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_offset=0;%exp_data.actPos(tmp_idx(1)-10);
        u_offset=0;%exp_data.actCur(tmp_idx(1)-10);
        % use 50 ms of data after step high for G2
        ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+69)-y_offset;
        utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+69)-u_offset;
        % visualize
        close(figure(200))
        figure(200);
        %         set(gcf, 'Position', get(0, 'Screensize'));
        hold on;
        plot(t,simOut.yout{3}.Values.Data(1:end-1),"k");
        plot(t,simOut.yout{2}.Values.Data(1:10:end-1),"b");
        plot(t,y2,"r");
        plot(t,ytmp,"g");
        plot(t,utmp,"--g");
        RMSE=sqrt(sum((ytmp(50:end)-y2(50:end)).^2));
        title(append("RMSE=",string(RMSE),", P=",string(gains(1)),", D=",string(gains(2))))
        %         for metrics
        y_high=ytmp(50:end); %todo check
        t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
        y_init=mean(exp_data.actPos((tmp_idx(1)-60):(tmp_idx(1)-10)))-y_offset;
        y_final=mean(exp_data.actPos((tmp_idx(end)-60):(tmp_idx(end)-10)))-y_offset;
        % manually calculate settling time for server because server lsiminfo is wrong
        i_st = max(find(abs(y_high-y_final)>0.02*(y_final-y_init)));
        st_gt=t_high(i_st+1);
        if isnan(st_gt)
            st_gt=3;
        end
        if max(y_high)>reference
            ov_gt=max(0,(max(y_high)-y_init)/(y_final-y_init)-1);
        else
            ov_gt=0;
        end
        Tr_gt=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
        e_gt=y_high-reference;
        ITAE_gt = trapz(t_high(1:ceil(5*Tr*1000)), abs(e_gt(1:ceil(5*Tr*1000))));
        e_ss_gt=abs(y_final-reference);
        metric_hat_all=[metric_hat_all;[ov, Tr, st, ITAE, e_ss]]
        metric_gt_all=[metric_gt_all;[ov_gt, Tr_gt, st_gt, ITAE_gt, e_ss_gt]]
        metric_error_all=[metric_error_all;[(ov-ov_gt)/ov_gt*100, (Tr-Tr_gt)/Tr_gt*100, (st-st_gt)/st_gt*100, (ITAE-ITAE_gt)/ITAE_gt*100,(e_ss-e_ss_gt)/e_ss_gt*100]]
        save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/metric_debug.mat","metric_error_all","metric_gt_all","metric_hat_all")
        pause;
        %%
        clc
        step_high=40;
        fw=65;
        bw=5;
        gains_gt=[5.341250000000000e+03,49.6875];
        exp_data=LinMotor(gains_gt(1),gains_gt(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_offset=0;%exp_data.actPos(tmp_idx(1)-10);
        y_gt = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;


        i=1; %iteration after N0
        gains_BO=[5581.25000000000	44.6875000000000];
        gains_GBO=[5581.25000000000	44.6875000000000];


        %         gains_BO=[5.131250000000000e+03,46.1875];
        %         gains_BO=[6128.75000000000	40.8125000000000];
        exp_data=LinMotor(gains_BO(1),gains_BO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_BO_1 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        %         gains_GBO=[5.3375e3,50.6250];
        %         gains_GBO=[5138.75000000000	49.3125000000000];
        exp_data=LinMotor(gains_GBO(1),gains_GBO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_GBO_1 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        close(figure(300))
        figure(300);
        subplot(2,2,1)
        hold on;
        t=0:0.001:(length(y_gt)-1)*0.001;
        plot(t,y_gt,"g");
        plot(t,y_BO_1,"b");
        plot(t,y_GBO_1,"r");

        i=2; %iteration after N0
        gains_BO=[6132.50000000000	50.8250000000000];
        gains_GBO=[5153.75000000000	50.5625000000000];


        %         gains_BO=[5.131250000000000e+03,46.1875];
        %         gains_BO=[6128.75000000000	40.8125000000000];
        exp_data=LinMotor(gains_BO(1),gains_BO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_BO_2 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        %         gains_GBO=[5.3375e3,50.6250];
        %         gains_GBO=[5138.75000000000	49.3125000000000];
        exp_data=LinMotor(gains_GBO(1),gains_GBO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_GBO_2 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        subplot(1,4,2)
        hold on;
        t=0:0.001:(length(y_gt)-1)*0.001;
        plot(t,y_gt,"g");
        plot(t,y_BO_2,"b");
        plot(t,y_GBO_2,"r");

        i=3; %iteration after N0
        gains_BO=[6125	50.7500000000000];
        gains_GBO=[5213.75000000000	50.5625000000000];


        %         gains_BO=[5.131250000000000e+03,46.1875];
        %         gains_BO=[6128.75000000000	40.8125000000000];
        exp_data=LinMotor(gains_BO(1),gains_BO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_BO_3 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        %         gains_GBO=[5.3375e3,50.6250];
        %         gains_GBO=[5138.75000000000	49.3125000000000];
        exp_data=LinMotor(gains_GBO(1),gains_GBO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_GBO_3 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        subplot(1,4,3)
        hold on;
        t=0:0.001:(length(y_gt)-1)*0.001;
        plot(t,y_gt,"g");
        plot(t,y_BO_3,"b");
        plot(t,y_GBO_3,"r");

        i=4; %iteration after N0
        gains_BO=[6125	50.7500000000000];
        gains_GBO=[5412.50000000000	50.8250000000000];

        %         gains_BO=[5.131250000000000e+03,46.1875];
        %         gains_BO=[6128.75000000000	40.8125000000000];
        exp_data=LinMotor(gains_BO(1),gains_BO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_BO_4 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        %         gains_GBO=[5.3375e3,50.6250];
        %         gains_GBO=[5138.75000000000	49.3125000000000];
        exp_data=LinMotor(gains_GBO(1),gains_GBO(2));
        sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_GBO_4 = exp_data.actPos((tmp_idx(1)-bw):tmp_idx(1)+fw)-y_offset;

        subplot(1,4,4)
        hold on;
        t=0:0.001:(length(y_gt)-1)*0.001;
        plot(t,y_gt,"g");
        plot(t,y_BO_4,"b");
        plot(t,y_GBO_4,"r");

        %%
        clc
        close(figure(400))
        fig=figure(400);
        fig.Position=[100 500 1200 750];
        ax1=subplot(2,2,1);
        %         ax1=axes;
        ax1.FontSize=24;
        ax1.FontName='Times New Roman';
        hold on
        step_high=40;
        step_low=30;
        h1=plot(t, y_gt, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
        h3=plot(t, y_GBO_1, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
        h4=plot(t, y_BO_1, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
        h5=stairs([0,bw*0.001,t(end)+0.001], [step_low, step_high,step_low],'--k', 'LineWidth', 1.5);
        grid on
        title("Iteration=1");
%         set(gca,'yscale','log')
        %         legend([h1, h3, h4, h5],{'Optimum', 'Guided BO', 'BO', 'Reference Input'}, 'Location', 'northeast');
        grid on
        xlim(ax1, [0,t(end)])
        yticks([step_low, step_high])
        %         xlabel(ax1, 'Time (s)')
        %         ylabel(ax1, 'Position (mm)')
        box on
        p1=get(ax1,'position');

        ax1=subplot(2,2,2);
        %         ax1=axes;
        ax1.FontSize=24;
        ax1.FontName='Times New Roman';
        hold on
        step_high=40;
        step_low=30;
        h1=plot(t, y_gt, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
        h3=plot(t, y_GBO_2, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
        h4=plot(t, y_BO_2, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
        h5=stairs([0,bw*0.001,t(end)+0.001], [step_low, step_high,step_low],'--k', 'LineWidth', 1.5);
        grid on
        title("Iteration=2");
        %         legend([h1, h3, h4, h5],{'Optimum', 'Guided BO', 'BO', 'Reference Input'}, 'Location', 'northeast');
        grid on
        xlim(ax1, [0,t(end)])
        yticks([step_low, step_high])
        %         xlabel(ax1, 'Time (s)')
        %         ylabel(ax1, 'Position (mm)')
        box on
        p2=get(ax1,'position');
        ax1=subplot(2,2,3);
        %         ax1=axes;
        ax1.FontSize=24;
        ax1.FontName='Times New Roman';
        hold on
        step_high=40;
        step_low=30;
        h1=plot(t, y_gt, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
        h3=plot(t, y_GBO_3, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
        h4=plot(t, y_BO_3, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
        h5=stairs([0,bw*0.001,t(end)+0.001], [step_low, step_high,step_low],'--k', 'LineWidth', 1.5);
        grid on
        title("Iteration=3");
        %         legend([h1, h3, h4, h5],{'Optimum', 'Guided BO', 'BO', 'Reference Input'}, 'Location', 'northeast');
        grid on
        xlim(ax1, [0,t(end)])
        yticks([step_low, step_high])
        %         xlabel(ax1, 'Time (s)')
        %         ylabel(ax1, 'Position (mm)')
        box on
        p3=get(ax1,'position');
        ax1=subplot(2,2,4);
        %         ax1=axes;
        ax1.FontSize=24;
        ax1.FontName='Times New Roman';
        hold on
        step_high=40;
        step_low=30;
        h1=plot(t, y_gt, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.5);
        h3=plot(t, y_GBO_4, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
        h4=plot(t, y_BO_4, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.5);
        h5=stairs([0,bw.*0.001,t(end)+0.001], [step_low, step_high,step_low],'--k', 'LineWidth', 1.5);
        grid on
        title("Iteration=4");
        legend([h1, h3, h4, h5],{'Optimum', 'Guided BO', 'BO', 'Reference Input'}, 'Location', 'northeast');
        grid on
        xlim(ax1, [0,t(end)])
        yticks([step_low, step_high])
        %         xlabel(ax1, 'Time (s)')
        %         ylabel(ax1, 'Position (mm)')
        box on
        p4=get(ax1,'position');
        height=p1(2)+p1(4)-p4(2);
        width=p4(1)+p4(3)-p3(1);
        h5=axes('position',[p3(1) p3(2) width height],'visible','off');
        h5.XLabel.Visible='on'
        h5.YLabel.Visible='on'
        axes(h5)
        ylabel('Position (mm)')
        xlabel('Time (s)')
        set(gca, 'FontName', 'Times New Roman', 'FontSize', 24)
        % Give common xlabel, ylabel and title to your figure
        %         han=axes(fig,'visible','off');
        %         han.Title.Visible='on';
        %         han.XLabel.Visible='on';
        %         han.YLabel.Visible='on';
        %         ylabel(han,'Time (s)',"FontSize",24,"FontName",'Times New Roman');
        %         xlabel(han,'Position (mm)',"FontSize",24,"FontName",'Times New Roman');
        %         set(gca,'TickLabelInterpreter','latex');
        %         xlh.Position(2) = xlh.Position(2) + abs(xlh.Position(2) * 0.01);
        %         set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
        %                 ax1.FontSize=24;
        %         ax1.FontName='Times New Roman';
        %         title(han,'yourTitle');
        %%
        figName="/home/mahdi/ETHZ/GBO/LM_response_exp_42_iter_1to4.png";
        saveas(gcf,figName)

    end
end
if isnan(ov) || isinf(ov) || ov>1
    ov=1;
end
if isnan(st) || isinf(st) || st>70e-3
    st=70e-3;
end
if isnan(Tr) || isinf(Tr) || Tr>70e-3
    Tr=70e-3;
end
if isnan(ITAE) || isinf(ITAE) || ITAE>1
    ITAE=1;
end
if isnan(e_ss) || isinf(e_ss) || e_ss>10
    e_ss=10;
end
if Tr==0
    Tr=sampleTs;
end
if ITAE==0
    ITAE=sampleTs*(reference-reference0);
end
if st==0
    st=sampleTs;
end

w_mean_grid=[0.0732, 0.0425, 0.0117, 0.2044];%, 0.0339]; %grid mean of feasible set mean(perf_Data_feasible)
% w_importance=[1.2, 1.05, 0.98, 1, 1.1];
% w_mean_grid=[0.0425, 0.0117, 0.2044]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1, 1.05, 1.1, 1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=st*w(1)+Tr*w(2)+ITAE*w(3);
end
%%
function [objective] = ObjFun_Guided(X, surrogate)
global N
global G2data
global N_G2
global sampleTs
N=N+1;
if surrogate==true
    npG2=2;
    nzG2=1;
    Options = tfestOptions('Display','off');
    Options.InitialCondition = 'backcast';
    Options.EnforceStability=1;
    G2v = tfest(G2data, npG2,nzG2,Options, 'Ts', sampleTs);
    z = tf('z',sampleTs);
    G2 = G2v * sampleTs/2 * (z+1)/(z-1);
    objective=ObjFun([], G2, X);
    N_G2=N_G2+1;
elseif surrogate==false
    exp_data=LinMotor(X(1), X(2));
    objective = ObjFun(exp_data,[],[]);
    step_high=40;
    step_down=30;
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos(tmp_idx(1)-10);
    u_offset=exp_data.actCur(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    vtmp = diff(ytmp((50+3):end))./sampleTs;
    utmp = utmp((50+3):(end-1));
    G2data = merge(G2data, iddata(vtmp,utmp,sampleTs));
    save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/LM_201_debug/G2data.mat","G2data")
end
fprintf('N= %d , N_G2= %d , ', N, N_G2);
end
%% sample from LM offline dataset
function exp_data=LinMotor(Kp,Kd)
global exp_data_crop_safe
i=find((exp_data_crop_safe.P==Kp).*(exp_data_crop_safe.D==Kd));
exp_data.actPos=exp_data_crop_safe.actPos_all(:,i);
exp_data.actVel=exp_data_crop_safe.actVel_all(:,i);
exp_data.actCur=exp_data_crop_safe.actCur_all(:,i);
exp_data.r=exp_data_crop_safe.r;
exp_data.t=exp_data_crop_safe.t;
exp_data.P=exp_data_crop_safe.P(i);
exp_data.D=exp_data_crop_safe.D(i);
end