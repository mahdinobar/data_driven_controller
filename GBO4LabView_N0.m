% build and save initial dataset
global G2data
addpath("C:\Users\nobar\Documents\LabVIEW Data\functions")
addpath C:\Program Files\MATLAB\R2021b\toolbox\ident\ident\@iddata\iddata.m
dir="C:\Users\nobar\Documents\LabVIEW Data\N0_Data\";
start_switch=1;

N0=1; %for N0>1 modify
gains0=[0.0950, 1.3293];
Kp=gains0(1);
Ki=gains0(2);

step_low=10;
step_high=20;
step_time=6;
nr_repeats=2;
control_mode=1;
Input_mode=2;

gain_angle=0;
Tn_Angle=0;

if counter==-1
gain_vel=Kp;
Tn_vel=1/Ki;
stat_value=0.5;
Input_mode=0;
counter=counter+1;
return
end

if counter==0
gain_vel=Kp;
Tn_vel=1/Ki;
counter=counter+1;
return
end

sampleTs=1/100;
if counter==N0
    ytmp = exp_Data(:,3);
    utmp= exp_Data(:,4);
    G2data_init = iddata(ytmp,utmp,sampleTs);
    J_init=ObjFun(perf_Data(end,:));
    botrace0.samples=[Kp, Ki];
    botrace0.values=J_init;
    botrace0.times=0;
    save(append(dir, 'G2data_init'),'G2data_init');
    save(append(dir, 'botrace0'), 'botrace0');
    save(append(dir, 'gains0'), 'gains0');
    save(append(dir, 'perf_Data'), 'perf_Data');
    save(append(dir, 'exp_Data'), 'exp_Data');
end
return
