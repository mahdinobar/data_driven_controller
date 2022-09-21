%  build initial N0 dataset
idx=0;
LV_switch=0;
% build and save initial dataset
global G2data
addpath("C:\Users\nobar\Documents\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\@iddata\iddata.m
dir="C:\Users\nobar\Documents\LabVIEW Data\N0_Data_1\";
if not(isfolder(dir))
    mkdir(dir)
end
start_switch=1;
stat_value=60;

N0=1; %for N0>1 modify
% gains0=[0.5, 1.47]; %initial random 
gains0=[0.5349, 1.0692]; %initial random 
% gains0=[0.3349, 1.5692]; %initial random 
% gains0=[0.4873, 1.5970]; %nominal for PM 90degree and GM=49db
Kp=gains0(1);
Ki=gains0(2);

sampleTf=2.5;
sampleTs=0.01;
Nsample=sampleTf/sampleTs;
% sampleTf=2.5;%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% Nsample=150;
% sampleTs=sampleTf/(Nsample-1);

step_low=80;
step_high=120;
step_time=5;
nr_repeats=4; % if you decrease nr_repeats to 2 you must modify J_init too
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

if counter<1
    gains0=[0.5, 1.47]; %initial random
    Kp=gains0(1);
    Ki=gains0(2);
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
return
