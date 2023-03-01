hyper_grid=zeros(2,2);
%  build initial N0 dataset
LV_switch=0;
LVswitch=0;
counter_s=0;
counter_real=0;
idx_G2=[0];
% build and save initial dataset
addpath("C:\mahdi\data_driven_controller\functions")
addpath C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\@iddata\iddata.m
tmp_name="exper_72_debug_2";
dir=append("C:\mahdi\Data\",tmp_name,"\N0_Data_",string(expr),"\");
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
stat_value=60;

N0=1; %for N0>1 modify
Kp=gains0(1);
Ki=gains0(2);

sampleTf=2.5;
sampleTs=0.01;
Nsample=sampleTf/sampleTs;

step_low=80;
step_high=120;
step_time=5;
nr_repeats=2; % if you decrease nr_repeats to 2 you must modify J_init too
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

if counter<1
    gains0_init=[0.5, 1.47]; %initial random
    Kp=gains0_init(1);
    Ki=gains0_init(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    Input_mode=2;
    counter=counter+1;
    return
end

if counter==1
    gain_vel=Kp;
    Tn_vel=1/Ki;
    counter=counter+1;
    return
end

sampleTs=1/100;
if counter==N0+1
    sample_idx=exp_Data(:,3)==step_high;
    ytmp = exp_Data(sample_idx,3);
    utmp= exp_Data(sample_idx,4);
    ytmp = ytmp(1:Nsample);
    utmp = utmp(1:Nsample);
    G2data_init = iddata(ytmp,utmp,sampleTs);

    J_init=ObjFun(perf_Data(end-nr_repeats+1:end,:));
    botrace0.samples=[Kp, Ki];
    botrace0.values=J_init;
    botrace0.times=0;
    save(append(dir, 'G2data_init'),'G2data_init');
    save(append(dir, 'botrace0'), 'botrace0');
    save(append(dir, 'gains0'), 'gains0');
    save(append(dir, 'perf_Data'), 'perf_Data');
    save(append(dir, 'exp_Data'), 'exp_Data');
end
counter=counter+1;
if counter>N0+1
    expr=expr+1;
    counter=0;
    dir_gains=append('C:\mahdi\data_driven_controller\Data\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
    load(dir_gains)
    gains0=[Kp_min+rand(1,1)*(Kp_max-Kp_min),Ki_min+rand(1,1)*(Ki_max-Ki_min)];
end
return
% %%%%%%%%%%%%%%%%%%%%%
% mkdir("C:\mahdi\data_driven_controller\Data\TEST000000000000000000000000\")
% %%%%%%%%%%%%%%%%%%%%%