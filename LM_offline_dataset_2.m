%% clear start and get single array of data
clear all; close all; clc
t = (0.001:0.001:7);
r= 10.*(t>2)+30-10.*(t>5);
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_offline_dataset_1.mat')
exp_data_1_1=exp_data;
clearvars exp_data
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_offline_dataset_1_rest.mat')
exp_data_1_2=exp_data;
clearvars exp_data
exp_data_all.actPos_all=[exp_data_1_1.actPos_all,exp_data_1_2.actPos_all];
exp_data_all.actCur_all=[exp_data_1_1.actCur_all,exp_data_1_2.actCur_all];
exp_data_all.actVel_all=[exp_data_1_1.actVel_all,exp_data_1_2.actVel_all];
exp_data_all.r=r;
exp_data_all.t=t;
exp_data_all.P=[exp_data_1_1.P_all,exp_data_1_2.P_all];
exp_data_all.D=[exp_data_1_1.D_all,exp_data_1_2.D_all];
clearvars exp_data_1_1 exp_data_1_2

for i=2:8
    load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_offline_dataset_',string(i),'.mat'))
    exp_data_tmp=exp_data;
    clearvars exp_data
    exp_data_all.actPos_all=[exp_data_all.actPos_all,exp_data_tmp.actPos_all];
    exp_data_all.actCur_all=[exp_data_all.actCur_all,exp_data_tmp.actCur_all];
    exp_data_all.actVel_all=[exp_data_all.actVel_all,exp_data_tmp.actVel_all];
    exp_data_all.r=r;
    exp_data_all.t=t;
    exp_data_all.P=[exp_data_all.P,exp_data_tmp.P_all];
    exp_data_all.D=[exp_data_all.D,exp_data_tmp.D_all];
end
for i=102:2:116
    load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_offline_dataset_',string(i),'.mat'))
    exp_data_tmp=exp_data;
    clearvars exp_data
    exp_data_all.actPos_all=[exp_data_all.actPos_all,exp_data_tmp.actPos_all];
    exp_data_all.actCur_all=[exp_data_all.actCur_all,exp_data_tmp.actCur_all];
    exp_data_all.actVel_all=[exp_data_all.actVel_all,exp_data_tmp.actVel_all];
    exp_data_all.r=r;
    exp_data_all.t=t;
    exp_data_all.P=[exp_data_all.P,exp_data_tmp.P_all];
    exp_data_all.D=[exp_data_all.D,exp_data_tmp.D_all];
end

%%
perf_Data_feasible=[];
P_safe=[];
PM_feasible=[];
D_safe=[];
P_unsafe=[];
D_unsafe=[];
objective_feasible=[];
sampleTs=0.001;
step_high=40;
y_high_all=[];
t_high_all=[];
idx_unsafe=[];
for exper=1:1:length(exp_data_all.P)
    exper
    step_high=40;
    sample_idx=exp_data_all.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data_all.actPos_all(tmp_idx(1)-10,exper);
    u_offset=exp_data_all.actCur_all(tmp_idx(1)-10,exper);
    % use 50 ms of data after step high for G2
    ytmp = exp_data_all.actPos_all((tmp_idx(1)-50):tmp_idx(1)+70,exper)-y_offset;
    utmp = exp_data_all.actCur_all((tmp_idx(1)-50):tmp_idx(1)+70,exper)-u_offset;
    reference0=0;
    reference=10;
    y_high=ytmp(50:end); %todo check
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_init=mean(exp_data_all.actPos_all((tmp_idx(1)-60):(tmp_idx(1)-10),exper))-y_offset;
    y_final=mean(exp_data_all.actPos_all((tmp_idx(end)-60):(tmp_idx(end)-10),exper))-y_offset;
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
    
    if ITAE==0 || st==0
        perf_Data=[-1,-1,-1,-1,-1];
        P_unsafe=[P_unsafe;exp_data_all.P(exper)];
        D_unsafe=[D_unsafe;exp_data_all.D(exper)];
        idx_unsafe=[idx_unsafe;exper];
    else
        perf_Data=[ov,Tr,st,ITAE,e_ss];
        objective = ObjFun(perf_Data);
        objective_feasible=[objective_feasible;objective];
        perf_Data_feasible=[perf_Data_feasible;perf_Data];
        P_safe=[P_safe;exp_data_all.P(exper)];
        D_safe=[D_safe;exp_data_all.D(exper)];
    end
end
% get safe data
exp_data_safe=exp_data_all;
exp_data_safe.actPos_all(:,idx_unsafe)=[];
exp_data_safe.actCur_all(:,idx_unsafe)=[];
exp_data_safe.actVel_all(:,idx_unsafe)=[];
exp_data_safe.P(idx_unsafe)=[];
exp_data_safe.D(idx_unsafe)=[];

save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/offline_data_tmp_3.mat")
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data_3.mat","exp_data_all","P_safe","D_safe","exp_data_safe","idx_unsafe","P_unsafe","D_unsafe")
%% debug
clear all; clc
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_KpKd_bounds.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_KpKd_bounds.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data_3.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/offline_data_tmp_3.mat")

idx_crop_safe=logical((P_safe<Kp_max).*(P_safe>Kp_min).*(D_safe<Kd_max).*(D_safe>Kd_min));
P_crop_safe=P_safe(idx_crop_safe);
D_crop_safe=D_safe(idx_crop_safe);
exp_data_crop_safe=exp_data_safe;
exp_data_crop_safe.objective_feasible=objective_feasible;
exp_data_crop_safe.perf_Data_feasible=perf_Data_feasible;
exp_data_crop_safe.acrtPos_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actCur_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actVel_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.P(~idx_crop_safe)=[];
exp_data_crop_safe.D(~idx_crop_safe)=[];
exp_data_crop_safe.objective_feasible(~idx_crop_safe)=[];
exp_data_crop_safe.perf_Data_feasible(~idx_crop_safe,:)=[];
% save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_crop_safe.mat")
%% plot J_hat vs gains using surrogate
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data.mat")
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/LM_201_debug/G2data.mat")
npG2=2;
nzG2=1;
sampleTs=0.001;
Options = tfestOptions('Display','off');
Options.InitialCondition = 'backcast';
Options.EnforceStability=1;
G2v = tfest(G2data, npG2,nzG2,Options, 'Ts', sampleTs);
z = tf('z',sampleTs);
G2 = G2v * sampleTs/2 * (z+1)/(z-1);
objective_feasible_hat=[];
perf_Data_feasible_hat=[];
for k=1:length(exp_data_crop_safe.P)
    k
%     P=P_safe(k);
%     D=D_safe(k);
%     F=0.001;
%     s = tf('s');
%     F=0.001;
%     Ptmp=P;
%     Dtmp=D;
%     C=Ptmp+Dtmp*s/(F*s+1);
%     Ts = sampleTs;
%     CL=feedback(d2c(G2)*C, 1);
%     isstable(CL)
%     reference0=0;
%     reference=10;
%     t_high=(51*Ts):Ts:(0.120-Ts);
%     t_down=0:Ts:(50*Ts);
%     step_high=reference.*ones(length(t_high),1);
%     step_down=reference0.*ones(length(t_down),1);
%     t=[t_down,t_high]';
%     r=[step_down;step_high];
%     y2=lsim(CL,r,t);
%     y_high=y2(t>(.01)); %TODO check pay attention
%     t_high=t(t>(.01));%TODO check 

    F=0.001;
    P=exp_data_crop_safe.P(k)/512;
    D=exp_data_crop_safe.D(k)/768;
    I=0;
    reference0=0;
    reference=10;
    G2c=d2c(G2);
    G2_num=G2c.Numerator{1};
    G2_den=G2c.Denominator{1};

    mdlWks = get_param('DT','ModelWorkspace');
    assignin(mdlWks,'sampleTs',sampleTs)
    assignin(mdlWks,'P',P)
    assignin(mdlWks,'D',D)
    assignin(mdlWks,'I',I)
    assignin(mdlWks,'F',F)
    assignin(mdlWks,'reference0',reference0)
    assignin(mdlWks,'reference',reference)
    assignin(mdlWks,'G2_den',G2_den)
    assignin(mdlWks,'G2_num',G2_num)
    simOut = sim("DT.slx");

    y2=simOut.yout{1}.Values.Data(1:10:end-1);
    t=simOut.tout(1:10:end-1);
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

    perf_Data_hat=[ov,Tr,st,ITAE,e_ss];
    perf_Data_feasible_hat=[perf_Data_feasible_hat;perf_Data_hat];
    objective_hat = ObjFun(perf_Data_hat);
    objective_feasible_hat=[objective_feasible_hat;objective_hat];
end
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/LM_201_debug/debug.mat")
%%
% figure(30)
hold on
% set(gca,'Zscale','log')
% set(gca,'ColorScale','log')
% h_infeasible=scatter3(P_unsafe,D_unsafe,max(objective_feasible_hat).*ones(size(D_unsafe)),20,"filled","r");
% h_feasible=scatter3(P_safe,D_safe,max(objective_feasible_hat).*ones(size(D_safe)),20,"filled","g");
% [m,I]=min(objective_feasible_hat);
% h_min=scatter3(P_safe(I),D_safe(I),max(objective_feasible_hat),300,"pentagram","filled","y");

x=P_crop_safe;
y=D_crop_safe;
z=objective_feasible_hat;
% plot3(x,y,z,"ok")
[xi,yi] = meshgrid(min(x):1:max(x), min(y):0.0167:max(y));
zi = griddata(x,y,z,xi,yi);
% [c,h]=contour(xi,yi,zi,10);
% clabel(c,h);
h_hat=surf(xi,yi,zi);

% colorbar
xlabel("P")
ylabel("D")
zlabel("J")
ylim([41,51])
% legend([h_feasible,h_infeasible, h_min, h_hat],["feasible","experimental failure", "optimum", "objectiveHAT"])
%%
figure(1)
subplot(5,1,1)
plot(perf_Data_feasible(:,1)./mean(perf_Data_feasible(:,1)),"-")
ylabel('overshoot')
title("Normalized metrics")
subplot(5,1,2)
plot(perf_Data_feasible(:,2)./mean(perf_Data_feasible(:,2)),"-")
% ylim([0,0.02])
ylabel('rise time')
subplot(5,1,3)
plot(perf_Data_feasible(:,3)./mean(perf_Data_feasible(:,3)),"-")
ylabel('settling time')
subplot(5,1,4)
plot(perf_Data_feasible(:,4)./mean(perf_Data_feasible(:,4)),"-")
ylabel('ITAE')
subplot(5,1,5)
plot(perf_Data_feasible(:,5)./mean(perf_Data_feasible(:,5)),"-")
ylabel('|e_ss|')

%%
figure(10)
subplot(5,1,1)
plot(perf_Data_feasible(:,1),"-")
ylabel('overshoot')
title("Estimated metrics")
subplot(5,1,2)
plot(perf_Data_feasible(:,2),"-")
% ylim([0,0.02])
ylabel('rise time')
subplot(5,1,3)
plot(perf_Data_feasible(:,3),"-")
ylabel('settling time')
subplot(5,1,4)
plot(perf_Data_feasible(:,4),"-")
ylabel('ITAE')
subplot(5,1,5)
plot(perf_Data_feasible(:,5),"-")
ylabel('|e_ss|')

%%
figure(2)
subplot(3,2,1)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,3);
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "settling time"})
%
subplot(3,2,2)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,2);
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "rise time"})
%
subplot(3,2,3)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,1);
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "overshoot"})
%
subplot(3,2,4)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,4);
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "ITAE"})

subplot(3,2,5)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,5);
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],["feasible","experimental failure", "absolute steady state error"])

subplot(3,2,6)
hold on
h_infeasible=scatter(P_unsafe,D_unsafe,"filled","r");
h_feasible=scatter(P_safe,D_safe,"filled","g");
x=P_safe;
y=D_safe;
z=perf_Data_feasible(:,5)./(reference-reference0).*100;
[xi,yi] = meshgrid(min(x):10:max(x), min(y):10:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],["feasible","experimental failure", "relative percentage absolute ss error"])

%%
figure(3)
hold on
set(gca,'Zscale','log')
set(gca,'ColorScale','log')
% h_infeasible=scatter3(P_unsafe,D_unsafe,max(objective_feasible).*ones(size(D_unsafe)),20,"filled","r");
% h_feasible=scatter3(P_safe,D_safe,max(objective_feasible).*ones(size(D_safe)),20,"filled","g");
[m,I]=min(exp_data_crop_safe.objective_feasible);

x=exp_data_crop_safe.P;
y=exp_data_crop_safe.D;
z=exp_data_crop_safe.objective_feasible;
% plot3(x,y,z,"ok")
[xi,yi] = meshgrid(min(x):1:max(x), min(y):0.0167:max(y));
zi = griddata(x,y,z,xi,yi);
% [c,h]=contour(xi,yi,zi,10);
% clabel(c,h);
h=surf(xi,yi,zi,'EdgeColor', 'none');
h_min=scatter3(exp_data_crop_safe.P(I),exp_data_crop_safe.D(I),exp_data_crop_safe.objective_feasible(I),450,"pentagram","filled","y");
colorbar
xlabel("P")
ylabel("D")
zlabel("J")
ylim([41,51])
legend([h_min, h],["optimum", "objective"])
view(3)
% legend([h_min, h, h_hat],["optimum", "ground truth J", "J_{hat}"])

%%
perf_Data_feasible=[];
P_safe=[];
PM_feasible=[];
D_safe=[];
P_unsafe=[];
D_unsafe=[];
objective_feasible=[];
sampleTs=0.001;
step_high=40;
y_high_all=[];
t_high_all=[];
idx_unsafe=[];
for exper=1:length(exp_data_all.P)
    exper
    sample_idx=exp_data_all.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 0.2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data_all.actPos_all(tmp_idx(1)-10,exper);
    u_offset=exp_data_all.actCur_all(tmp_idx(1)-10,exper);
    ytmp = exp_data_all.actPos_all((tmp_idx(1)-10):tmp_idx(end),exper)-y_offset;
    utmp = exp_data_all.actCur_all((tmp_idx(1)-10):tmp_idx(end),exper)-u_offset;
    %     if exist('G2data')
    %         G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    %     else
    %         G2data = iddata(ytmp,utmp,sampleTs);
    %     end
    % G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    %calculate performance data based on experimental step response measurements
    reference0=0;
    reference=10;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    y_high_all=[y_high_all,y_high];
    t_high_all=[t_high_all,t_high];
    y_init=mean(exp_data_all.actPos_all((tmp_idx(1)-60):(tmp_idx(1)-10),exper))-y_offset;
    y_final=mean(exp_data_all.actPos_all((tmp_idx(end)-60):(tmp_idx(end)-10),exper))-y_offset;
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));

    e_ss=abs(y_final-reference);
    if ITAE==0 && st==0
        perf_Data=[-1,-1,-1,-1,-1];
        P_unsafe=[P_unsafe;exp_data_all.P(exper)];
        D_unsafe=[D_unsafe;exp_data_all.D(exper)];
        idx_unsafe=[idx_unsafe;exper];
    else
        perf_Data=[ov,Tr,st,ITAE,e_ss];
        objective = ObjFun2(exp_data_all);
        objective_feasible=[objective_feasible;objective];
        perf_Data_feasible=[perf_Data_feasible;perf_Data];
        P_safe=[P_safe;exp_data_all.P(exper)];
        D_safe=[D_safe;exp_data_all.D(exper)];

    end
end
% get safe data
exp_data_safe=exp_data_all;
exp_data_safe.actPos_all(:,idx_unsafe)=[];
exp_data_safe.actCur_all(:,idx_unsafe)=[];
exp_data_safe.actVel_all(:,idx_unsafe)=[];
exp_data_safe.P(idx_unsafe)=[];
exp_data_safe.D(idx_unsafe)=[];

% save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/offline_data_tmp.mat")
% save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data.mat","exp_data_all","P_safe","D_safe","exp_data_safe","idx_unsafe","P_unsafe","D_unsafe")


%% functions
function objective = ObjFun(perf_Data)

% TODO only use first experiment per gains
ov=abs(perf_Data(1,1));
st=perf_Data(1,3);
Tr=perf_Data(1,2);
ITAE = perf_Data(1,4);
e_ss = perf_Data(1,5);
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

% w_mean_grid=[0.0732, 0.0425, 0.0117, 0.2044, 0.0339]; %grid mean of feasible set mean(perf_Data_feasible)
% w_importance=[1.2, 1.05, 0.98, 1, 1.1];
w_mean_grid=[0.0425, 0.0117, 0.2044]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1.05, 1.1, 1];
w=w_importance./w_mean_grid; 
w=w./sum(w);
objective=st*w(1)+Tr*w(2)+ITAE*w(3);

end

%%
function [objective] = ObjFun2(exp_data)
step_high=40;
sampleTs=0.001;
sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
tmp_idx=find(sample_idx>0);
tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
tmp_idx=tmp_idx(tmp_idx_2);
y_offset=exp_data.actPos_all(tmp_idx(1)-10);
u_offset=exp_data.actCur_all(tmp_idx(1)-10);
% use 50 ms of data after step high for G2
ytmp = exp_data.actPos_all((tmp_idx(1)-100):tmp_idx(1)+50)-y_offset;
utmp = exp_data.actCur_all((tmp_idx(1)-100):tmp_idx(1)+50)-u_offset;

reference0=0;
reference=10;
y_high=ytmp(10:end);
t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
y_init=mean(exp_data.actPos_all((tmp_idx(1)-60):(tmp_idx(1)-10)))-y_offset;
y_final=mean(exp_data.actPos_all((tmp_idx(end)-5):(tmp_idx(end))))-y_offset;
S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
st=S.SettlingTime;
if isnan(st)
    st=3;
end
ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
e=abs(y_high-reference);
ITAE = trapz(t_high(1:ceil(3*Tr*1000)), t_high(1:ceil(3*Tr*1000)).*abs(e(1:ceil(3*Tr*1000))));
e_ss=abs(y_final-reference);
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
w_mean_grid=[0.1506, 0.0178, 0.0940, 0.0190, 0.4968]; %grid mean of feasible set mean(perf_Data_feasible)
w_importance=[1.2, 1.05, 0.98, 1, 1.1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)+e_ss*w(5);
end



