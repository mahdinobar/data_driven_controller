%% functions
function objective = ObjFun(perf_Data)
% if size(perf_Data,1)>1
%     ov=mean(abs(perf_Data(:,1)));
%     st=mean(perf_Data(:,3));
%     Tr=mean(perf_Data(:,2));
%     ITAE = mean(perf_Data(:,4));
% else
%     ov=abs(perf_Data(1,1));
%     st=perf_Data(1,3);
%     Tr=perf_Data(1,2);
%     ITAE = perf_Data(1,4);
% end

% TODO only use first experiment per gains
ov=abs(perf_Data(1,1));
st=perf_Data(1,3);
Tr=perf_Data(1,2);
ITAE = perf_Data(1,4);

if isnan(ov) || isinf(ov) || ov>1
    ov=1;
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

% w_mean_grid=[0.272170491516590,3.10390673875809,0.368857250362635,31.5501121520996]; %based on mean values of 10 initial dataset performance measurements at C:\mahdi\data_driven_controller\Data\objective_w_gains_estimation\
w_mean_grid=[0.2074, 2.4504, 0.2820, 20.0302]; %grid mean of feasible set with no >4.5 second settling time
% w_mean_grid=[10.5360, 3.8150, 0.6119, 1.1596];
% w_importance=[2, 1, 1, 1];
w_importance=[2, 5, 1, 1];
w=w_importance./w_mean_grid;
w=w./sum(w);
objective=ov*w(1)+st*w(2)+Tr*w(3)+ITAE*w(4);
end
