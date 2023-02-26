% %% functions
% 
% ov=mean(abs(perf_Data(:,1)));
% st=mean(perf_Data(:,3));
% Tr=mean(perf_Data(:,2));
% ITAE = mean(perf_Data(:,4));
% 
% if isnan(ov) || isinf(ov) || ov>1e3
%     ov=1e3;
% end
% 
% if isnan(st) || isinf(st) || st>1e5
%     st=1e5;
% end
% 
% if isnan(Tr) || isinf(Tr) || Tr>1e5
%     Tr=1e5;
% end
% 
% if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
%     ITAE=1e5;
% end
% 
% w=[2, 1, 1*0.15, 0.1*0.003];
% w=w./sum(w);
% objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4)
% 
close all
figure(1)
y=Trace.values(3:end);
h1=plot(y)
hold on
h2=plot(std(y).*ones(length(y),1),'--')
ylabel('cost')
legend([h1 h2],{'cost','std cost'});
title(append('std/mean % = ',num2str((std(y)/mean(y))*100),'%'))

n_trials=9;
ov=zeros(n_trials,1);
st=zeros(n_trials,1);
Tr=zeros(n_trials,1);
ITAE=zeros(n_trials,1);
ITAE_w=zeros(n_trials,1);
ov_w=zeros(n_trials,1);
st_w=zeros(n_trials,1);
Tr_w=zeros(n_trials,1);
for i=1:n_trials
    load(append('C:\Users\nobar\Documents\LabVIEW Data\BO_Data\demo_BO_new_test_20\perf_Data_',num2str(i+1)))
    [ov(i), st(i), Tr(i), ITAE(i), ov_w(i), st_w(i), Tr_w(i), ITAE_w(i)] = ObjFun_tmp(perf_Data);
end
metrics_w=[ov_w,st_w, Tr_w, ITAE_w];
h3=bar(metrics_w);
set(h3, {'DisplayName'}, {'weighted ov','weighted st','weighted Tr', 'weighted ITAE'}')

figure(2)
metrics=[ov,st, Tr, ITAE];
h4=bar(metrics);
legend(h4, {'ov','st','Tr', 'ITAE'}')


%% functions
function [ov, st, Tr, ITAE, ov_w, st_w, Tr_w, ITAE_w] = ObjFun_tmp(perf_Data)
ov=mean(abs(perf_Data(:,1)));
st=mean(perf_Data(:,3));
Tr=mean(perf_Data(:,2));
ITAE = mean(perf_Data(:,4));

if isnan(ov) || isinf(ov) || ov>1e3
    ov=1e3;
end

if isnan(st) || isinf(st) || st>1e5
    st=1e5;
end

if isnan(Tr) || isinf(Tr) || Tr>1e5
    Tr=1e5;
end

if isnan(ITAE) || isinf(ITAE) || ITAE>1e5
    ITAE=1e5;
end

%w0=[1, 1*0.1, 1*0.15, 0.1*0.003];
% w0=[1/0.0678, 1/0.8609, 1/0.0867, 1/18.3778];
% w_mean=[0.0517    1.7722    0.2967   57.5018];
w_mean=[0.2055    2.6423    0.4419   30.4598];
w0=w_mean.^(-1);

w=w0./sum(w0);
ov_w=ov*w(1);
st_w=+st*w(2);
Tr_w=Tr*w(3);
ITAE_w=ITAE*w(4);
end

