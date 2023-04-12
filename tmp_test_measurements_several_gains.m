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
tmp_name="tmp_test_measurements_several_gains";
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
else
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
    J_init=ObjFun(perf_Data);
    botrace0.samples=[Kp, Ki];
    botrace0.values=J_init;
    botrace0.times=0;
    save(append(dir, 'y_offset_',num2str(counter-1),'.mat'),'y_offset');
    save(append(dir, 'u_offset_',num2str(counter-1),'.mat'),'u_offset');    
    save(append(dir, 'G2data_',num2str(counter-1),'.mat'),'G2data');
    save(append(dir, 'botrace0_',num2str(counter-1),'.mat'), 'botrace0');
    save(append(dir, 'gains0_',num2str(counter-1),'.mat'), 'gains0');
    save(append(dir, 'perf_Data_',num2str(counter-1),'.mat'), 'perf_Data');
    save(append(dir, 'exp_Data_',num2str(counter-1),'.mat'), 'exp_Data');
end
counter=counter+1;
if counter>11
    counter=1;
    dir_gains=append('C:\mahdi\data_driven_controller\Data\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
    load(dir_gains)
    gains_span=10;

%     choose grid Kp,Ki from low to high and return back from high to
%     low(works for expr in [1:200]
    expr_tmp=expr;
    if expr<101
            expr_tmp=expr;
    else
            expr_tmp=expr-100;
    end
    tmp=linspace(0,1,gains_span);
    if rem(expr_tmp,gains_span)==0
        idx_tmp=10;
    else
        idx_tmp=rem(expr_tmp,gains_span);
    end
    gains0=[Kp_min+tmp(idx_tmp)*(Kp_max-Kp_min),Ki_min+tmp(floor((expr_tmp-1)/gains_span)+1)*(Ki_max-Ki_min)];

    expr=expr+1;
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