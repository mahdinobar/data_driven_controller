%%
clear all; clc; close all;
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data.mat")
%%
figure(1)
hold on
plot(exp_data.t_all(:,1),exp_data.r_all(:,1),'k');
plot(exp_data.t_all(:,1),exp_data.actPos_all(:,1:200));
xlabel('Time [s]'); ylabel('Pos [mm]')
colormap(jet(200))
hc = colorbar;
cb = linspace(1,200,200);
set(hc, 'YTick',cb, 'YTickLabel',cb)


figure(2)
hold on
plot(exp_data.t_all(:,1),exp_data.actVel_all);
xlabel('Time [s]'); ylabel('Vel [m/s]')


figure(3)
hold on
plot(exp_data.t_all(:,1),exp_data.actCur_all);
xlabel('Time [s]'); ylabel('Cur')


%%
perf_Data_all=[];
y_offset=30.0300;
u_offset=-0.2100;
sampleTs=0.001;
step_high=40;
for exper=2:200
    % y_offset=exp_data.actPos_all(2000,exper);
    % u_offset=exp_data.actCur_all(2000,exper);
    sample_idx=exp_data.r_all(:,1)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    ytmp = exp_data.actPos_all((tmp_idx(1)-10):tmp_idx(end),exper)-y_offset;
    utmp = exp_data.actCur_all((tmp_idx(1)-10):tmp_idx(end),exper)-u_offset;
    if exist('G2data')
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    else
        G2data = iddata(ytmp,utmp,sampleTs);
    end
    % G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    %calculate performance data based on experimental step response measurements
    reference0=0;
    reference=10;
    y_high=ytmp(10:end);
    t_high=0:sampleTs:((length(y_high)-1)*sampleTs);
    S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
    Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));
    perf_Data=[ov,Tr,st,ITAE];
    perf_Data_all=[perf_Data_all;perf_Data];
end
figure(4)
subplot(4,1,1)
plot(perf_Data_all(:,1))
ylabel('overshoot')
subplot(4,1,2)
plot(perf_Data_all(:,2))
ylim([0,0.02])
ylabel('rise time')
subplot(4,1,3)
plot(perf_Data_all(:,3))
ylabel('settling time')
subplot(4,1,4)
plot(perf_Data_all(:,4))
ylabel('ITAE')
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/perf_Data_all.mat","perf_Data_all")
%%
close figure 3
figure(3)
plot(perf_Data_all(:,3))
plot(t_high,y_high)
figure(3)
hold on
plot(exp_data.t_all(:,1),exp_data.actCur_all);
xlabel('Time [s]'); ylabel('Cur')
%% plot cost in feasible set
clc;
clear all;
close all;
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_feasible.mat','exp_data')
perf_Data_feasible=[];
P_feasible=[];
D_feasible=[];
P_infeasible=[];
D_infeasible=[];
% y_offset=30.0300;
% u_offset=-0.2100;
sampleTs=0.001;
step_high=40;
y_high_all=[];
t_high_all=[];
for exper=1:length(exp_data.P)
    exper
    if exp_data.t_all(1,exper)~=-1
        sample_idx=exp_data.r_all(:,1)==step_high; %LV sampling time=10 ms
        tmp_idx=find(sample_idx>0);
        tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 0.2 seconds
        tmp_idx=tmp_idx(tmp_idx_2);
        y_offset=exp_data.actPos_all(tmp_idx(1)-10,exper);
        u_offset=exp_data.actCur_all(tmp_idx(1)-10,exper);
        ytmp = exp_data.actPos_all((tmp_idx(1)-10):tmp_idx(end),exper)-y_offset;
        utmp = exp_data.actCur_all((tmp_idx(1)-10):tmp_idx(end),exper)-u_offset;
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
        S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.02);
        st=S.SettlingTime;
        if isnan(st)
            st=3;
        end
        ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
        Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
        e=abs(y_high-reference);
        ITAE = trapz(t_high(1:ceil(5*Tr*1000)), t_high(1:ceil(5*Tr*1000))'.*abs(e(1:ceil(5*Tr*1000))));
        perf_Data=[ov,Tr,st,ITAE];
        perf_Data_feasible=[perf_Data_feasible;perf_Data];
        P_feasible=[P_feasible;exp_data.P(exper)];
        D_feasible=[D_feasible;exp_data.D(exper)];
    else
        P_infeasible=[P_infeasible;exp_data.P(exper)];
        D_infeasible=[D_infeasible;exp_data.D(exper)];
    end
end
figure(5)
subplot(4,1,1)
plot(perf_Data_feasible(:,1))
ylabel('overshoot')
subplot(4,1,2)
plot(perf_Data_feasible(:,2))
ylim([0,0.02])
ylabel('rise time')
subplot(4,1,3)
plot(perf_Data_feasible(:,3))
ylabel('settling time')
subplot(4,1,4)
plot(perf_Data_feasible(:,4))
ylabel('ITAE')
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/Data_feasible.mat","perf_Data_feasible","P_feasible","D_feasible")
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/Data_infeasible.mat","P_infeasible","D_infeasible")
%%
close
figure(1)
hold on
xlabel("P")
ylabel("D")
% scatter(exp_data.P,exp_data.D,"filled","k")
h_infeasible=scatter(P_infeasible,D_infeasible,"filled","r");
h_feasible=scatter(P_feasible,D_feasible,"filled","g");
legend([h_feasible,h_infeasible],{"feasible","infeasible (PM<20)"})
xlim([500,12000])
ylim([10,90])
xticks([500, 2000:2000:12000])
%%
close
figure(6)
hold on
% scatter(exp_data.P,exp_data.D,"filled","k")
scatter(P_infeasible,D_infeasible,"filled","r");
h=surf(P_feasible,D_feasible,perf_Data_feasible(:,3));
% legend([h_feasible,h_infeasible],{"feasible","infeasible (PM<20)"})
xlim([500,12000])
ylim([10,90])
xticks([500, 2000:2000:12000])
xlabel("P")
ylabel("D")
%%
close
figure(7)
subplot(2,2,1)
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
legend([h_feasible,h_infeasible, h],{"feasible","infeasible (PM<20)", "settling time"})
%
subplot(2,2,2)
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
legend([h_feasible,h_infeasible, h],{"feasible","infeasible (PM<20)", "rise time"})
%
subplot(2,2,3)
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
legend([h_feasible,h_infeasible, h],{"feasible","infeasible (PM<20)", "overshoot"})
%
subplot(2,2,4)
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
legend([h_feasible,h_infeasible, h],{"feasible","infeasible (PM<20)", "ITAE"})
