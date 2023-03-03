% % find failed experiments
% nan_expr=[];
% all_perf=[];
% values=[];
% for i=1:74
%     load(append('C:\mahdi\data_driven_controller\Data\exper_72_debug_2\N0_Data_',string(i),'\perf_Data','.mat'))
%     load(append('C:\mahdi\data_driven_controller\Data\exper_72_debug_2\N0_Data_',string(i),'\botrace0','.mat'))
%     nan_perf=sum(isnan(perf_Data(:,1:4)));
%     all_perf=[all_perf;perf_Data];
%     values=[values;botrace0.values];
%     if nan_perf>1
%         nan_expr=[nan_expr,i];
%     end
% end
% nan_expr'
% 

% clc
% close all
% t=1/100:1/100:5;
% sample_idx=exp_Data(:,3)==120;
% y = exp_Data(sample_idx,4);
% y2 = 120*step(G2,t);
% rmse2=sqrt(mean((y-y2).^2))
% plot(y,'b');hold on; grid on; plot(y2,'r')

DATA=[];
for i=1:9
    load(append('C:\mahdi\data_driven_controller\Data\exper_72\BO_',string(i),'\trace_file_expr_',string(i)','.mat'))
    alpha=2;
    data=[max(Trace.post_sigma2s), mean(Trace.post_sigma2s), std(Trace.post_sigma2s), sum(Trace.post_sigma2s>alpha*std(Trace.post_sigma2s)+mean(Trace.post_sigma2s)), alpha*std(Trace.post_sigma2s)+mean(Trace.post_sigma2s)];
    DATA=[DATA;data];
end
DATA
mean(DATA,1)
