rehash 
hyper_grid=zeros(2,2);
%  build initial N0 dataset
LVswitch=0;
counter_s=0;
counter_real=0;
idx_G2=[0];
% build and save initial dataset
addpath("C:\mahdi\data_driven_controller\functions")
addpath C:\Program Files\MATLAB\R2022b\toolbox\ident\ident\@iddata\iddata.m
tmp_name="exper_72_5";
dir=append("C:\mahdi\data_driven_controller\Data\",tmp_name,"\N0_Data_",string(expr),"\");
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
stat_value=60; 

N0=1; %for N0>1 modify
Kp=gains0(1);
Ki=gains0(2);

% sampleTf=2.5;
% sampleTs=0.01;
% Nsample=sampleTf/sampleTs;
sampleTs=10e-3; % 10ms

step_low=80;
step_high=120;
step_time=5;
nr_repeats=2; % if you decrease nr_repeats to 2 you must modify J_init too
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

%% to initialize first the response
if counter==0  %global initialize counter from 0
    Kp=0.5;
    Ki=1.47;
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
elseif counter==1
    Kp=gains0(1);
    Ki=gains0(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
elseif counter==2
%     %%%%%%%%%%%%%%%%%%%%%
%     save(append(dir, 'debug_expr.mat'));
%     %%%%%%%%%%%%%%%%%%%%%
    sample_idx=exp_Data(:,3)==step_high;
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_Data(tmp_idx(1)-10,4);
    u_offset=exp_Data(tmp_idx(1)-10,5);
    ytmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),4)-y_offset;
    utmp = exp_Data((tmp_idx(1)-10):tmp_idx(end),5)-u_offset;
    G2data = iddata(ytmp,utmp,sampleTs);
    %calculate performance data based on experimental step response measurements
    reference0=0;
    reference=40;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    e=abs(y_high-reference);
    ITAE = trapz(t_high, t_high'.*abs(e));
    S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.05);
    st=S.SettlingTime;
    if isnan(st)
        st=5;
    end
    ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
    Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
    perf_Data=[ov,Tr,st,ITAE];

    J_init=ObjFun(perf_Data);
    botrace0.samples=[Kp, Ki];
    botrace0.values=J_init;
    botrace0.times=0;
    save(append(dir, 'y_offset.mat'),'y_offset');
    save(append(dir, 'u_offset.mat'),'u_offset');    
    save(append(dir, 'G2data'),'G2data');
    save(append(dir, 'botrace0'), 'botrace0');
    save(append(dir, 'gains0'), 'gains0');
    save(append(dir, 'perf_Data'), 'perf_Data');
    save(append(dir, 'perf_Data_LV'), 'perf_Data_LV');
    save(append(dir, 'exp_Data'), 'exp_Data');
end
counter=counter+1;
if counter>2
    expr=expr+1;
    counter=1;
    dir_gains=append('C:\mahdi\data_driven_controller\Data\DC_motor_gain_bounds\KpKi_bounds_new_3.mat');
    load(dir_gains)
    gains0=[Kp_min+rand(1,1)*(Kp_max-Kp_min),Ki_min+rand(1,1)*(Ki_max-Ki_min)];
    Kp=0.5;
    Ki=1.47;
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
end

return
% %%%%%%%%%%%%%%%%%%%%%
% mkdir("C:\mahdi\data_driven_controller\Data\TEST000000000000000000000000\")
% %%%%%%%%%%%%%%%%%%%%%