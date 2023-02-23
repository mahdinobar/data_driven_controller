%% functions
function objective = ObjFun(perf_Data)
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

% w=[1, 1*0.1, 1*0.15, 0.1*0.003];
% w=[1/0.0678, 1/0.8609, 1/0.0867, 1/18.3778];
% w_mean=[0.0517    1.7722    0.2967   57.5018];
% w_mean=[0.2055    2.6423    0.4419   30.4598];
% w0=w_mean.^(-1);
% w=w0./sum(w0);
% objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);
% w_mean_grid=[10.5360, 3.8150, 0.6119, 1.1596];
w_mean_grid=[0.2097, 2.6459, 0.4352, 30.9839]; % based on C:\Users\nobar\Documents\LabVIEW Data\N0_Data_set_obj_weights_20
w_importance=[2, 1, 1, 1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);
end
