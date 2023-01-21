function draft()

    for j=1:N_G2_tmp
        load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_9/G2_', num2str(num2str(j))),'G2')
        sigma2_s=sigma2_s+1/(n-1)*(ObjFun(X, G2, false)-y_gt)^2; 
    end

end

