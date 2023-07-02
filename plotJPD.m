%% load and prepare LM offline dataset
load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_offline_data.mat")
dir_gains='/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/LM_KpKd_bounds.mat';
load(dir_gains)
idx_crop_safe=logical((P_safe<Kp_max).*(P_safe>Kp_min).*(D_safe<Kd_max).*(D_safe>Kd_min));
P_crop_safe=P_safe(idx_crop_safe);
D_crop_safe=D_safe(idx_crop_safe);

exp_data_crop_safe=exp_data_safe;
exp_data_crop_safe.actPos_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actCur_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.actVel_all(:,~idx_crop_safe)=[];
exp_data_crop_safe.P(~idx_crop_safe)=[];
exp_data_crop_safe.D(~idx_crop_safe)=[];

J=[];
for k=1:length(P_crop_safe)
    k
    exp_data.actPos_all=exp_data_crop_safe.actPos_all(:,k);
    exp_data.actCur_all=exp_data_crop_safe.actCur_all(:,k);
    exp_data.actVel_all=exp_data_crop_safe.actVel_all(:,k);
    exp_data.P=exp_data_crop_safe.P(k);
    exp_data.D=exp_data_crop_safe.D(k);
    exp_data.t=exp_data_crop_safe.t;
    exp_data.r=exp_data_crop_safe.r;
    J=[J;ObjFun(exp_data, [], [])];
end
%%
figure(1)
hold on
set(gca,'Zscale','log')
set(gca,'ColorScale','log')
h_infeasible=scatter3(P_unsafe,D_unsafe,max(J).*ones(size(D_unsafe)),20,"filled","r");
h_feasible=scatter3(P_safe,D_safe,max(J).*ones(size(D_safe)),20,"filled","g");
[m,I]=min(J);
h_min=scatter3(P_safe(I),D_safe(I),max(J),300,"pentagram","filled","y");

x=P_crop_safe;
y=D_crop_safe;
z=J;
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
ylim([41,51])
legend([h_feasible,h_infeasible, h_min, h],["feasible","experimental failure", "optimum", "objective"])

%%
function [objective] = ObjFun(exp_data, G2, gains)
step_high=40;
step_down=30;
sampleTs=0.001;
if isempty(G2)==1
    sample_idx=exp_data.r(:)==step_high; %LV sampling time=10 ms
    tmp_idx=find(sample_idx>0);
    tmp_idx_2=find(tmp_idx>200); %checkpoint because we know step_up applies no sooner than 2 seconds
    tmp_idx=tmp_idx(tmp_idx_2);
    y_offset=exp_data.actPos_all(tmp_idx(1)-10);
    u_offset=exp_data.actCur_all(tmp_idx(1)-10);
    % use 50 ms of data after step high for G2
    ytmp = exp_data.actPos_all((tmp_idx(1)-50):tmp_idx(1)+70)-y_offset;
    utmp = exp_data.actCur_all((tmp_idx(1)-50):tmp_idx(1)+70)-u_offset;
    if exist('G2data')
        G2data = merge(G2data, iddata(ytmp,utmp,sampleTs));
    else
        G2data = iddata(ytmp,utmp,sampleTs);
    end
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
    ITAE = trapz(t_high(1:ceil(3*Tr*1000)), t_high(1:ceil(3*Tr*1000))'.*abs(e(1:ceil(3*Tr*1000))));
    e_ss=abs(y_final-reference);
elseif isempty(G2)==0 %when we use surrogate to estimate objective
    P=gains(1);
    D=gains(2);
    F=0.001;

    s = tf('s');
    F=0.001;
    Ptmp=P;
    Dtmp=D;
    C=Ptmp+Dtmp*s/(F*s+1);

%     Kp = P;
%     Ti = inf;
%     Td = D/P;
%     N=D/(P*F);
    Ts = sampleTs;
%     C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
    CL=feedback(d2c(G2)*C, 1);
    isstable(CL)
    reference0=0;
    reference=10;
    %%
    t_high=(51*Ts):Ts:(0.120-Ts);
    t_down=0:Ts:(50*Ts);
    step_high=reference.*ones(length(t_high),1);
    step_down=reference0.*ones(length(t_down),1);
    t=[t_down,t_high]';
    r=[step_down;step_high];
    y2=lsim(CL,r,t);
%     %%
%     s = tf('s');
%     F=0.001;
%     Ptmp=P;
%     Dtmp=D;
%     C22=Ptmp+Dtmp*s/(F*s+1);
%     C22d = c2d(C22,Ts);
%     CL22d=feedback(G2*C22d, 1);
%     isstable(CL22d)
%     y22d=lsim(CL22d,r,t);
%     %%
%     s = tf('s');
%     F=0.001;
%     Ptmp=P;
%     Dtmp=D;
%     C22=Ptmp+Dtmp*s/(F*s+1);
%     CL22=feedback(d2c(G2)*C22, 1);
%     isstable(CL22)
%     y22=lsim(CL22,r,t);
    %%
    y_high=y2(t>(.01)); %TODO check pay attention
    t_high=t(t>(.01));%TODO check    
    y_init=0;
    y_final=mean(y_high(end-5:end));
    e_ss=abs(y_final-reference);
    S = lsiminfo(y_high,t_high,y_final,y_init,'SettlingTimeThreshold',0.02);
    st=S.SettlingTime;
    if isnan(st)
        st=3;
    end
    ov=max(0,(S.Max-y_init)/(y_final-y_init)-1);
    Tr=t_high(find(y_high>0.6*(y_final-y_init),1))-t_high(find(y_high>0.1*(y_final-y_init),1));
    e=abs(y_high-reference);
    ITAE = trapz(t_high(1:ceil(3*Tr*1000))', t_high(1:ceil(3*Tr*1000)).*abs(e(1:ceil(3*Tr*1000))));
    e_ss=abs(y_final-reference);
end
if isnan(ov) || isinf(ov) || ov>1
    ov=1;
end
if isnan(st) || isinf(st) || st>3
    st=3;
end
if isnan(Tr) || isinf(Tr) || Tr>3
    Tr=3;
end
if isnan(ITAE) || isinf(ITAE) || ITAE>30
    ITAE=30;
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
