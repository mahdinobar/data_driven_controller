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
P_feasible=[];
PM_feasible=[];
D_feasible=[];
P_infeasible=[];
D_infeasible=[];
objective_feasible=[];
sampleTs=0.001;
step_high=40;
y_high_all=[];
t_high_all=[];
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
            P_infeasible=[P_infeasible;exp_data_all.P(exper)];
            D_infeasible=[D_infeasible;exp_data_all.D(exper)];
        else
            perf_Data=[ov,Tr,st,ITAE,e_ss];
            objective = ObjFun(perf_Data);
            objective_feasible=[objective_feasible;objective];
            perf_Data_feasible=[perf_Data_feasible;perf_Data];
            P_feasible=[P_feasible;exp_data_all.P(exper)];
            D_feasible=[D_feasible;exp_data_all.D(exper)];
        
    end
end
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/offline_data_tmp.mat")
%%
figure(1)
subplot(5,1,1)
plot(perf_Data_feasible(:,1)./mean(perf_Data_feasible(:,1)),"o")
ylabel('overshoot')
subplot(5,1,2)
plot(perf_Data_feasible(:,2)./mean(perf_Data_feasible(:,2)),"o")
% ylim([0,0.02])
ylabel('rise time')
subplot(5,1,3)
plot(perf_Data_feasible(:,3)./mean(perf_Data_feasible(:,3)),"o")
ylabel('settling time')
subplot(5,1,4)
plot(perf_Data_feasible(:,4)./mean(perf_Data_feasible(:,4)),"o")
ylabel('ITAE')
subplot(5,1,5)
plot(perf_Data_feasible(:,5)./mean(perf_Data_feasible(:,5)),"o")
ylabel('|e_ss|')

%%
figure(2)
subplot(3,2,1)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,3);
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "settling time"})
%
subplot(3,2,2)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,2);
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "rise time"})
%
subplot(3,2,3)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,1);
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "overshoot"})
%
subplot(3,2,4)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g"); 
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,4);
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],{"feasible","experimental failure", "ITAE"})

subplot(3,2,5)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,5);
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],["feasible","experimental failure", "absolute steady state error"])

subplot(3,2,6)
hold on
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
x=P_feasible;
y=D_feasible;
z=perf_Data_feasible(:,5)./(reference-reference0).*100;
[xi,yi] = meshgrid(min(x):1:max(x), min(y):1:max(y));
zi = griddata(x,y,z,xi,yi);
[c,h]=contour(xi,yi,zi);
clabel(c,h);
xlabel("P")
ylabel("D")
legend([h_feasible,h_infeasible, h],["feasible","experimental failure", "relative percentage absolute ss error"])

%%
close
figure(3)
hold on
set(gca,'Zscale','log')
set(gca,'ColorScale','log')
h_infeasible=scatter3(P_infeasible,D_infeasible,max(objective_feasible).*ones(size(D_infeasible)),20,"filled","r");
h_feasible=scatter3(P_feasible,D_feasible,max(objective_feasible).*ones(size(D_feasible)),20,"filled","g");
[m,I]=min(objective_feasible);
h_min=scatter3(P_feasible(I),D_feasible(I),max(objective_feasible),300,"pentagram","filled","y");

x=P_feasible;
y=D_feasible;
z=objective_feasible;
% plot3(x,y,z,"ok")
[xi,yi] = meshgrid(min(x):1:max(x), min(y):0.0167:max(y));
zi = griddata(x,y,z,xi,yi);
% [c,h]=contour(xi,yi,zi,10);
% clabel(c,h);
h=surf(xi,yi,zi,'EdgeColor', 'none');
colorbar
xlabel("P")
ylabel("D")
zlabel("J")
legend([h_feasible,h_infeasible, h_min, h],["feasible","experimental failure", "optimum", "objective"])


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

if isnan(st) || isinf(st) || st>1e5
    st=3;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=3;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=30;
end

if isnan(e_ss) || isinf(e_ss) || e_ss>1e5
    e_ss=10;
end

% w_mean_grid=[0.272170491516590,3.10390673875809,0.368857250362635,31.5501121520996]; %based on mean values of 10 initial dataset performance measurements at C:\mahdi\data_driven_controller\Data\objective_w_gains_estimation\

w_mean_grid=[0.1506, 0.0178, 0.0940, 0.0190, 0.4968]; %grid mean of feasible set mean(perf_Data_feasible)
% w_mean_grid=[0.5605    0.1030    0.7213    0.3829    2.0497];% normalization values for max of each metric
% w_importance=[1.02, 1.02, 1.0, 1.0, 1];
w_importance=[1.2, 1.05, 0.98, 1, 1.1];
w=w_importance./w_mean_grid;
w=w./sum(w);
% w=[0.1,0.2,0.3,0.2,0.3];
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)+e_ss*w(5);
end




