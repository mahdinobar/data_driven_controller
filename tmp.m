% % % % %% functions
% % % %
% % % % ov=mean(abs(perf_Data(:,1)));
% % % % st=mean(perf_Data(:,3));
% % % % Tr=mean(perf_Data(:,2));
% % % % ITAE = mean(perf_Data(:,4));
% % % %
% % % % if isnan(ov) || isinf(ov) || ov>1e3
% % % %     ov=1e3;
% % % % end
% % % %
% % % % if isnan(st) || isinf(st) || st>1e5
% % % %     st=1e5;
% % % % end
% % % %
% % % % if isnan(Tr) || isinf(Tr) || Tr>1e5
% % % %     Tr=1e5;
% % % % end
% % % %
% % % % if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
% % % %     ITAE=1e5;
% % % % end
% % % %
% % % % w=[2, 1, 1*0.15, 0.1*0.003];
% % % % w=w./sum(w);
% % % % objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)
% % % %
% % % close all
% % % figure(1)
% % % y=Trace.values(3:end);
% % % h1=plot(y)
% % % hold on
% % % h2=plot(std(y).*ones(length(y),1),'--')
% % % ylabel('cost')
% % % legend([h1 h2],{'cost','std cost'});
% % % title(append('std/mean % = ',num2str((std(y)/mean(y))*100),'%'))
% % %
% % % n_trials=9;
% % % ov=zeros(n_trials,1);
% % % st=zeros(n_trials,1);
% % % Tr=zeros(n_trials,1);
% % % ITAE=zeros(n_trials,1);
% % % ITAE_w=zeros(n_trials,1);
% % % ov_w=zeros(n_trials,1);
% % % st_w=zeros(n_trials,1);
% % % Tr_w=zeros(n_trials,1);
% % % for i=1:n_trials
% % %     load(append('C:\Users\nobar\Documents\LabVIEW Data\BO_Data\demo_BO_new_test_20\perf_Data_',num2str(i+1)))
% % %     [ov(i), st(i), Tr(i), ITAE(i), ov_w(i), st_w(i), Tr_w(i), ITAE_w(i)] = ObjFun_tmp(perf_Data);
% % % end
% % % metrics_w=[ov_w,st_w, Tr_w, ITAE_w];
% % % h3=bar(metrics_w);
% % % set(h3, {'DisplayName'}, {'weighted ov','weighted st','weighted Tr', 'weighted ITAE'}')
% % %
% % % figure(2)
% % % metrics=[ov,st, Tr, ITAE];
% % % h4=bar(metrics);
% % % legend(h4, {'ov','st','Tr', 'ITAE'}')
% % %
% % %
% % % %% functions
% % % function [ov, st, Tr, ITAE, ov_w, st_w, Tr_w, ITAE_w] = ObjFun_tmp(perf_Data)
% % % ov=mean(abs(perf_Data(:,1)));
% % % st=mean(perf_Data(:,3));
% % % Tr=mean(perf_Data(:,2));
% % % ITAE = mean(perf_Data(:,4));
% % %
% % % if isnan(ov) || isinf(ov) || ov>1e3
% % %     ov=1e3;
% % % end
% % %
% % % if isnan(st) || isinf(st) || st>1e5
% % %     st=1e5;
% % % end
% % %
% % % if isnan(Tr) || isinf(Tr) || Tr>1e5
% % %     Tr=1e5;
% % % end
% % %
% % % if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
% % %     ITAE=1e5;
% % % end
% % %
% % % %w0=[1, 1*0.1, 1*0.15, 0.1*0.003];
% % % % w0=[1/0.0678, 1/0.8609, 1/0.0867, 1/18.3778];
% % % % w_mean=[0.0517    1.7722    0.2967   57.5018];
% % % w_mean=[0.2055    2.6423    0.4419   30.4598];
% % % w0=w_mean.^(-1);
% % %
% % % w=w0./sum(w0);
% % % ov_w=ov*w(1);
% % % st_w=+st*w(2);
% % % Tr_w=Tr*w(3);
% % % ITAE_w=ITAE*w(4);
% % % end
% % %
% % %
% % 
% % 
% % clc
% % close all
% % 
% % % npG2=2;
% % % num = [9.54434];
% % % den = [1, 4.14479, 4.19941];
% % % Td=2e-3;
% % % G2 = tf(num, den, 'InputDelay',Td);
% % 
% % % load('when_switch_s.mat')
% % % load('idx_G2.mat')
% % % load('trace_file_expr_1.mat')
% % 
% % % i=1;
% % % load(append('exp_Data_',num2str(when_switch_s(i)),'_1.mat'))
% % % load('G2_all_3.mat')
% % % gains=Trace.samples(when_switch_s(i),:);
% % % gains=botrace0.samples;
% % gains=Trace.samples(1,:);
% % step_high=120;
% % step_low=80;
% % % sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
% % % y = exp_Data(sample_idx,4);
% % % t=1:(length(y));
% % % t=0.01.*t;
% % 
% % % exp2=getexp(G2data,2);
% % 
% % sample_idx=exp_Data(:,3)==120;
% % ytmp = exp_Data(sample_idx,4);
% % utmp= exp_Data(sample_idx,5);
% % rtmp= exp_Data(sample_idx,3);
% % 
% % C=tf([gains(1), gains(1)*gains(2)], [1, 0]);
% % CL=feedback(C*G2, 1);
% % 
% % % exp2=iddata(ytmp,rtmp,0.01);
% % 
% % % G2_ss = idss(G2);
% % % X0 = findstates(G2_ss,exp2);
% % % [sys,ic] = tfest(exp2,2);
% % % opt = compareOptions('InitialCondition',ic);
% % % compare(exp2,CL)
% % % hold on
% % % ud=exp2.u;
% % % t=0:(length(ud)-1);
% % % t=t*0.01;
% % % yd=lsim(CL,ud,t);
% % % plot(t,yd,'--r')
% % 
% % 
% % 
% % 
% % % r = [sz
% % 
% % y2 = lsim(CL,rtmp,t);
% % y2=y2(T>5);
% % gains2=Trace.samples(5,:);
% % C2=tf([gains2(1), gains2(1)*gains2(2)], [1, 0]);
% % CL2=feedback(C2*G2, 1);
% % y3 = lsim(CL2,r,t);
% % 
% % figure(2)
% % % rmse2=sqrt(mean((y-y2).^2))
% % plot(ytmp,'b');hold on; plot(y2,'r'); %plot(y3,'k')
% % ylabel("y")
% % xlabel("timestep")
% % title("plant output vs sampling timestep")
% % 
% 
% % % %% functions
% % %
% % % ov=mean(abs(perf_Data(:,1)));
% % % st=mean(perf_Data(:,3));
% % % Tr=mean(perf_Data(:,2));
% % % ITAE = mean(perf_Data(:,4));
% % %
% % % if isnan(ov) || isinf(ov) || ov>1e3
% % %     ov=1e3;
% % % end
% % %
% % % if isnan(st) || isinf(st) || st>1e5
% % %     st=1e5;
% % % end
% % %
% % % if isnan(Tr) || isinf(Tr) || Tr>1e5
% % %     Tr=1e5;
% % % end
% % %
% % % if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
% % %     ITAE=1e5;
% % % end
% % %
% % % w=[2, 1, 1*0.15, 0.1*0.003];
% % % w=w./sum(w);
% % % objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)
% % %
% % close all
% % figure(1)
% % y=Trace.values(3:end);
% % h1=plot(y)
% % hold on
% % h2=plot(std(y).*ones(length(y),1),'--')
% % ylabel('cost')
% % legend([h1 h2],{'cost','std cost'});
% % title(append('std/mean % = ',num2str((std(y)/mean(y))*100),'%'))
% %
% % n_trials=9;
% % ov=zeros(n_trials,1);
% % st=zeros(n_trials,1);
% % Tr=zeros(n_trials,1);
% % ITAE=zeros(n_trials,1);
% % ITAE_w=zeros(n_trials,1);
% % ov_w=zeros(n_trials,1);
% % st_w=zeros(n_trials,1);
% % Tr_w=zeros(n_trials,1);
% % for i=1:n_trials
% %     load(append('C:\Users\nobar\Documents\LabVIEW Data\BO_Data\demo_BO_new_test_20\perf_Data_',num2str(i+1)))
% %     [ov(i), st(i), Tr(i), ITAE(i), ov_w(i), st_w(i), Tr_w(i), ITAE_w(i)] = ObjFun_tmp(perf_Data);
% % end
% % metrics_w=[ov_w,st_w, Tr_w, ITAE_w];
% % h3=bar(metrics_w);
% % set(h3, {'DisplayName'}, {'weighted ov','weighted st','weighted Tr', 'weighted ITAE'}')
% %
% % figure(2)
% % metrics=[ov,st, Tr, ITAE];
% % h4=bar(metrics);
% % legend(h4, {'ov','st','Tr', 'ITAE'}')
% %
% %
% % %% functions
% % function [ov, st, Tr, ITAE, ov_w, st_w, Tr_w, ITAE_w] = ObjFun_tmp(perf_Data)
% % ov=mean(abs(perf_Data(:,1)));
% % st=mean(perf_Data(:,3));
% % Tr=mean(perf_Data(:,2));
% % ITAE = mean(perf_Data(:,4));
% %
% % if isnan(ov) || isinf(ov) || ov>1e3
% %     ov=1e3;
% % end
% %
% % if isnan(st) || isinf(st) || st>1e5
% %     st=1e5;
% % end
% %
% % if isnan(Tr) || isinf(Tr) || Tr>1e5
% %     Tr=1e5;
% % end
% %
% % if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
% %     ITAE=1e5;
% % end
% %
% % %w0=[1, 1*0.1, 1*0.15, 0.1*0.003];
% % % w0=[1/0.0678, 1/0.8609, 1/0.0867, 1/18.3778];
% % % w_mean=[0.0517    1.7722    0.2967   57.5018];
% % w_mean=[0.2055    2.6423    0.4419   30.4598];
% % w0=w_mean.^(-1);
% %
% % w=w0./sum(w0);
% % ov_w=ov*w(1);
% % st_w=+st*w(2);
% % Tr_w=Tr*w(3);
% % ITAE_w=ITAE*w(4);
% % end
% %
% %
% 
% 
% clc
% close all
% % clear
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/botrace0.mat")
% % Trace=botrace0;
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/G2data_init.mat")
% % G2data=G2data_init;
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/exp_Data.mat")
% 
% npG2=2;
% num = [9.54434];
% den = [1, 4.14479, 4.19941];
% Td=2e-3;
% Gn = tf(num, den, 'InputDelay',Td);
% 
% G2=tfest(G2data, 2);
% 
% % init_sys = idtf(zeros(1,2),[1,zeros(1,2)],'IODelay',0,'Ts', 0.01);
% % init_sys.Structure.IODelay.Free = 0;
% % init_sys.Structure.IODelay.Maximum = 30;
% Options = tfestOptions('Display','on');
% Options.InitialCondition = 'estimate';
% Options.EnforceStability=1;
% G2d_WS = tfest(G2data_c_UpDown_0b_all_2, 2,1,Options, 'Ts', 0.01);
% 
% u_all=exp_Data(1:end,5);
% y_all=exp_Data(1:end,4);
% G2data_all=iddata(y_all,u_all,10e-3);
% G2_all=tfest(G2data_all, 2);
% 
% gains=Trace.samples(1,:);
% step_high=120;
% step_low=80;
% sample_idx=exp_Data(:,3)==step_high; %LV sampling time=10 ms
% y = exp_Data(sample_idx,4);
% u = exp_Data(sample_idx,5);
% t=1:(length(y));
% t=0.01.*t;
% 
% clear
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/exp_Data.mat")
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_2/G2data_c_UpDown_0b_all_2.mat")
% sample_idx=exp_Data(:,3)==120; %LV sampling time=10 ms
% tmp_idx=find(sample_idx>0);
% y_corrected = exp_Data((tmp_idx(1)-10):end,4)-exp_Data(tmp_idx(1)-1,4);
% u_corrected = exp_Data((tmp_idx(1)-10):end,5)-exp_Data(tmp_idx(1)-1,5);
% % u_corrected=u_corrected(1:10:end);
% % y_corrected=y_corrected(1:10:end);
% G2data_c_UpDown_0b_all_2=iddata(y_corrected,u_corrected,10e-3);
% % G2data_c_UpDown_0b_all_2=merge(G2data_c_UpDown_0b_all_2, iddata(y_corrected,u_corrected,10e-3));
% save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_2/G2data_c_UpDown_0b_all_2.mat","G2data_c_UpDown_0b_all_2")
% 
% exper=2;
% for i=1:1
%     load(append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_",num2str(exper),"/exp_Data_",num2str(exper),"_",num2str(i),".mat"))
%     load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_2/G2data_c_UpDown_0b_all_2.mat")
%     sample_idx=exp_Data(:,3)==120; %LV sampling time=10 ms
%     tmp_idx=find(sample_idx>0);
%     y_corrected = exp_Data((tmp_idx(1)-10):end,4)-exp_Data(tmp_idx(1)-1,4);
%     u_corrected = exp_Data((tmp_idx(1)-10):end,5)-exp_Data(tmp_idx(1)-1,5);
% %     u_corrected=u_corrected(1:10:end);
% %     y_corrected=y_corrected(1:10:end);
%     % G2data_c_UpDown_0b_all_2=iddata(y_corrected,u_corrected,10e-3);
%     G2data_c_UpDown_0b_all_2=merge(G2data_c_UpDown_0b_all_2, iddata(y_corrected,u_corrected,10e-3));
%     save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_2/G2data_c_UpDown_0b_all_2.mat","G2data_c_UpDown_0b_all_2")
% end
% 
% close
% exper=2;
% u_all=cell2mat(G2data_c_Up_0b_all.u);
% y_all=cell2mat(G2data_c_Up_0b_all.y);
% sampleTs=0.01;
% t=0:sampleTs:(length(u_all)-1)*sampleTs;
% lsim(tf,u_all(:,exper),t)
% hold on
% plot(t,y_all(:,exper),"g")
% %%
% Options = tfestOptions('Display','off');
% Options.InitialCondition = 'backcast';
% Options.EnforceStability=1;
% G2d_WS = tfest(G2data_c_UpDown_0b_all, 2,1,Options, 'Ts', 0.01);
% %%
% close
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/botrace0.mat")
% Trace=botrace0;
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/N0_Data_2/exp_Data.mat")
% G2=G2d_WS;
% sample_idx=exp_Data(:,3)==120; %LV sampling time=10 ms
% tmp_idx=find(sample_idx>0);
% y_all = exp_Data((tmp_idx(1)-10):end,4)-exp_Data(tmp_idx(1)-1,4);
% u_all = exp_Data((tmp_idx(1)-10):end,5)-exp_Data(tmp_idx(1)-1,5);
% r_all = exp_Data((tmp_idx(1)-10):end,3)-exp_Data(tmp_idx(1)-1,3);
% sampleTs=0.01;
% gains=Trace.samples(1,:);
% t=0:sampleTs:(length(u_all)-1)*sampleTs;
% % C=tf([gains(1), gains(1)*gains(2)], [1, 0], 'Ts', 0.01);
% Kp = gains(1);
% Ti = 1/gains(2);
% Td = 0;   
% N=inf;
% Ts = 0.01;
% C = pidstd(Kp,Ti,Td,N,Ts,'IFormula','Trapezoidal');
% CL=feedback(C*G2, 1);
% lsim(CL,r_all,t);
% y2=lsim(CL,r_all,t);
% reference0=0;
% reference=40;
% y_high=y2(t<5.1);
% t_high=t(t<5.1)';%TODO check
% e=abs(y_high-reference);
% ITAE = trapz(t_high, t_high.*abs(e));
% S = lsiminfo(y_high,t_high,reference,reference0);
% st=S.SettlingTime;
% if isnan(st) || st>5
%     st=5;
% end
% if ITAE>(reference-reference0)*40
%     ITAE=(reference-reference0)*40;
% end
% if isnan(Tr) || Tr>5
%     Tr=5;
% end
% if isnan(ov) || ov>100
%     ov=100;
% end
% ov=100.*max(0,(S.Max-reference0)/(reference-reference0)-1);
% Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
% perf_Data=[ov, Tr, st, ITAE]
% 
% hold on
% plot(t,y_all,"g")
% %%
% 
% uy_init_mean=[];
% uy_init_std=[];
% for exper=1:8
% uy_init=[];
% for i=1:50
%     load(append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/exper_72_3/GBO_",num2str(exper),"/exp_Data_",num2str(exper),"_",num2str(i)))
%     sample_idx=exp_Data(:,3)==120; %LV sampling time=10 ms
%     tmp_idx=find(sample_idx>0);
%     uy_init=[uy_init; exp_Data(tmp_idx(1)-1,5), exp_Data(tmp_idx(1)-1,4)];
% end
% uy_init_mean=[uy_init_mean;mean(uy_init)];
% uy_init_std=[uy_init_std;std(uy_init)];
% end
% 
% 
% C=tf([gains(1), gains(1)*gains(2)], [1, 0]);
% CL=feedback(C*G2, 1);
% CLn=feedback(C*Gn, 1);
% CL_all=feedback(C*G2_all, 1);
% 
% r = [step_low.*ones(length(t),1);step_high.*ones(length(t),1)];
% t0=t;
% t=t+5;
% T=[t0,t];
% 
% y2 = lsim(CL,r,T);
% y2=y2(T>5);
% 
% yn = lsim(CLn,r,T);
% yn=yn(T>5);
% 
% y_all = lsim(CL_all,r,T);
% y_all=y_all(T>5);
% 
% rmse2_s=sqrt(mean((y-y2).^2));
% rmse2_n=sqrt(mean((y-yn).^2));
% rmse2_all=sqrt(mean((y-y_all).^2));
% 
% FP=100*(1-norm(y-y2)/norm(y-mean(y)));
% 
% h1=plot(y,'b');hold on;
% h2=plot(y2,'r'); 
% hn=plot(yn,'k'); 
% h_all=plot(y_all,'--m'); 
% ylabel("y")
% xlabel("timestep")
% title(append("output vs timestep, rmse2_s=",num2str(rmse2_s), ", rmse2_n=",num2str(rmse2_n), ", rmse2_{all}=",num2str(rmse2_all)))
% grid on
% legend([h1,h2,hn, h_all],{"measurement","surrogate","high-fidelity plant","surrogate trained on all data"}, 'Location', 'southeast')
% 
% 
figure(1); hold on
for i=2:500
    load(append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/tmp_test_measurements/N0_Data_1/exp_Data_",string(i),".mat"))
    plot(exp_Data(:,4)); plot(exp_Data(:,3),'k');
end
figure(2); hold on
for i=2:500
    load(append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/tmp_test_measurements/N0_Data_1/exp_Data_",string(i),".mat"))
    plot(exp_Data(:,5)); plot(exp_Data(:,3),'k')
end