hyper_grid_pruned=zeros(2,2);
N_G2_activated_counter=0;
% build initial N0 dataset
idx=0;
LV_switch=0;
% build and save initial dataset
addpath("C:\mahdi\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2020b\toolbox\ident\ident\@iddata\iddata.m
dir_gains=append('C:\Users\students\Documents\data_driven_controller-main\data_driven_controller-main\tmp\DC_motor_gain_bounds\KpKi_bounds_new_2.mat');
dir=append("C:\mahdi\LabVIEW Data\N0_Data_",string(expr),"\");
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
stat_value=60;

N0=10; %for N0>1 modify
N_expr=50;
sampleTf=2.5;
sampleTs=0.01;
Nsample=sampleTf/sampleTs;
% sampleTf=2.5;%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% Nsample=150;
% sampleTs=sampleTf/(Nsample-1);

step_low=80;
step_high=120;
step_time=5;
nr_repeats=2; % if you decrease nr_repeats to 2 you must modify J_init too
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

if counter<1
    % sample from latin (denoted as ltn) hypercube
    RAND_ltn_all = sort(lhsdesign(N0,N_expr));
    save(append(dir,'RAND_ltn_all.mat'),'RAND_ltn_all')    
    load(dir_gains)
    gains0=[Kp_min+RAND_ltn_all.*(Kp_max-Kp_min);Ki_min+RAND_ltn_all.*(Ki_max-Ki_min)];
    save(append(dir,'gains0.mat'),'gains0')        
    gains0_init=[0.5, 1.47]; %initial random
    Kp=gains0_init(1);
    Ki=gains0_init(2);
    gain_vel=Kp;
    Tn_vel=1/Ki;
    step_low=40;
    step_high=40;
    nr_repeats=1;
    Input_mode=2;
    start_switch=1;
    counter=counter+1;
    LVswitch=0;
    return
else
    load(append(dir,'gains0.mat'),'gains0')        
    Kp=gains0(counter, expr);
    Ki=gains0(N0+counter, expr);
end

if LVswitch==0
    gain_vel=Kp;
    Tn_vel=1/Ki;
    LVswitch=LVswitch+1;
    return
elseif LVswitch==1
    LVswitch=0;
end

sample_idx=exp_Data(:,3)==step_high;
ytmp = exp_Data(sample_idx,3);
utmp= exp_Data(sample_idx,4);
ytmp = ytmp(1:Nsample);
utmp = utmp(1:Nsample);

if counter==1
    G2data_init = iddata(ytmp,utmp,sampleTs);
    save(append(dir, 'G2data_init.mat'),'G2data_init')
    J_init=ObjFun(perf_Data(end-nr_repeats+1:end,:));
    botrace0.samples=[Kp, Ki];
    botrace0.values=J_init;
    botrace0.times=0;
    save(append(dir, 'G2data_init'),'G2data_init');
    save(append(dir, 'botrace0'), 'botrace0');
else
    load(append(dir, 'G2data_init.mat'))
    G2data_init = merge(G2data_init, iddata(ytmp,utmp,sampleTs));
    save(append(dir, 'G2data_init.mat'),'G2data_init')
    load(append(dir, 'botrace0'));
    J_init=ObjFun(perf_Data(end-nr_repeats+1:end,:));
    botrace0.samples=[botrace0.samples;[Kp, Ki]];
    botrace0.values=[botrace0.values;J_init];
    botrace0.times=[botrace0.times;0];
    save(append(dir, 'botrace0'), 'botrace0');

end
save(append(dir, 'perf_Data_',num2str(counter),'_',num2str(expr)), 'perf_Data')
save(append(dir, 'exp_Data_',num2str(counter),'_',num2str(expr)), 'exp_Data')

counter=counter+1;
if counter>N0
    expr=expr+1;
    counter=0;
    %%%%%%%%%%%%%%%%%%%%%
    mkdir("C:\mahdi\LabVIEW Data\TEST0000000000000000000\")
    %%%%%%%%%%%%%%%%%%%%%
end
return
