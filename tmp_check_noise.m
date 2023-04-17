clear all
clc
close all


means=[];
stds=[];
means_LV=[];
stds_LV=[];
gains=[];
for n=1:200
    
    dir=append("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/tmp_test_measurements_several_gains/N0_Data_",string(n),"/");
    ov_all=[];
    Tr_all=[];
    st_all=[];
    ITAE_all=[];
    perf_Data_all_LV=[];
    load(append(dir,'gains0_1.mat'))
    for i=1:10
        load(append(dir,'G2data_',string(i),'.mat'))
        load(append(dir,'exp_Data_',string(i),'.mat'))
        load(append(dir, 'perf_Data_',string(i),'.mat'))

        Ts = 0.01;
        reference0=0;
        reference=40;

        y_high=G2data.y(10:end);
        t_high=0:Ts:((length(y_high)-1)*Ts);
        e=abs(y_high-reference);
        ITAE = trapz(t_high, t_high'.*abs(e));
        ITAE_all=[ITAE_all;ITAE];

        S = lsiminfo(y_high,t_high,reference,reference0,'SettlingTimeThreshold',0.05);
        st=S.SettlingTime;
        if isnan(st)
            st=5;
        end
        st_all=[st_all;st];

        ov=max(0,(S.Max-reference0)/(reference-reference0)-1);
        ov_all=[ov_all;ov];

        Tr=t_high(find(y_high>0.6*(reference-reference0),1))-t_high(find(y_high>0.1*(reference-reference0),1));
        Tr_all=[Tr_all;Tr];

        perf_Data_all_LV=[perf_Data_all_LV;perf_Data(1,1:4)];
    end
    perf_Data_all=[ov_all,Tr_all,st_all,ITAE_all];
    % remove NAN dat rows
    [r,~]=find(isnan(perf_Data_all));
    perf_Data_all(r,:)=[];
    perf_Data_all_LV(r,:)=[];

    means=[means;mean(perf_Data_all)];
    stds=[stds;std(perf_Data_all)];

    means_LV=[means_LV;mean(perf_Data_all)];
    stds_LV=[stds_LV;std(perf_Data_all)];
    gains=[gains;gains0];

end
save("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/tmp_test_measurements_several_gains/results.mat","gains","means","stds","means_LV","stds_LV")
selected_means=means(gains(:,1)>0.3291,:);
selected_stds=stds(gains(:,1)>0.3291,:);
selected_stds=selected_stds./mean(selected_means);
mean_selected_means=selected_means./mean(selected_means);
% %TODO manual corrections
% means(1,:)=[];
% means(101,:)=[];
% means_LV(1,:)=[];
% means_LV(101,:)=[];
% stds(1,:)=[];
% stds(101,:)=[];
% stds_LV(1,:)=[];
% stds_LV(101,:)=[];

figure(1)
subplot(4,1,1)
errorbar(means(2:100,1),stds(2:100,1),'b')
hold on
errorbar(means(102:200,1),stds(102:200,1),'r')
title({"overshoot"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,2)
errorbar(means(2:100,2),stds(2:100,2),'b')
hold on
errorbar(means(102:200,2),stds(102:200,2),'r')
title({"rise time"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,3)
errorbar(means(2:100,3),stds(2:100,3),'b')
hold on
errorbar(means(102:200,3),stds(102:200,3),'r')
title({"settling time"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,4)
errorbar(means(2:100,4),stds(2:100,4),'b')
hold on
errorbar(means(102:200,4),stds(102:200,4),'r')
title({"ITAE"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")

figure(2)
subplot(4,1,1)
errorbar(mean_selected_means(2:70,1),selected_stds(2:70,1),'b')
hold on
errorbar(mean_selected_means(72:140,1),selected_stds(72:140,1),'r')
title({"overshoot"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,2)
errorbar(mean_selected_means(2:70,2),selected_stds(2:70,2),'b')
hold on
errorbar(mean_selected_means(72:140,2),selected_stds(72:140,2),'r')
title({"rise time"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,3)
errorbar(mean_selected_means(2:70,3),selected_stds(2:70,3),'b')
hold on
errorbar(mean_selected_means(72:140,3),selected_stds(72:140,3),'r')
title({"settling time"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")
subplot(4,1,4)
errorbar(mean_selected_means(2:70,4),selected_stds(2:70,4),'b')
hold on
errorbar(mean_selected_means(72:140,4),selected_stds(72:140,4),'r')
title({"ITAE"});
legend({"first round","second round"});
xlabel("gains")
ylabel("metric")