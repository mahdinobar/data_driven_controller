perf=[];
for i=1:10
    load(append('C:\mahdi\data_driven_controller\Data\objective_w_gains_estimation\N0_Data_',num2str(i),'\perf_Data.mat'))
    perf=[perf;perf_Data];
end
perf
%% 