% GBO
for expr=1:54
    tmp_dir="/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/GBO_Experiment_data_26092022/";
    dir=append(tmp_dir,'/demo_GBO_', string(expr), '/');
    load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/idx_G2.mat'));
    load(append(dir, 'trace_file.mat'))
    Trace.samples(idx_G2,:)=[];
    Trace.values(idx_G2)=[];
    Trace.post_mus(idx_G2)=[];
    Trace.post_sigma2s(idx_G2)=[];
    Trace.times(idx_G2)=[];
    save(append(dir, 'trace_file_removed_2.mat'),'Trace')
end

return
