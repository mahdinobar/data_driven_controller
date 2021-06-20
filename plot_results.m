function plot_results
% hyper-params
idName= '15_13';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=5;
Nsample=10;
np2=2;
load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_15/15_13_InitData.mat')
load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_15/15_13_objectiveData_not_removed.mat')
load('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_15/15_13_objectiveEstData_not_removed.mat')
figure(1);hold on
plot(objectiveData_not_removed(N0+1:end), 'b','DisplayName','MinObjective')
if withSurrogate==true
    surrogate_iteration=1:N_surrogate_repeat+1:N_iter-N0;
    for i = 1:size(surrogate_iteration,2)
        xline(surrogate_iteration(i));
    end
end
plot(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
xlabel('iteration')
ylabel('objective')
ylim([-0.01 0.01])
xlim([1 N_iter-N0])
dir='/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/demo_15/';
figName=append(dir, idName, '.png');
% saveas(gcf,figName)

figure(2);hold on
semilogy(objectiveData_not_removed(N0+1:end), 'b','DisplayName','MinObjective')
if withSurrogate==true
    for i = 1:size(surrogate_iteration,2)
        xline(surrogate_iteration(i));
    end
end
semilogy(objectiveEstData_not_removed(N0+1:end), 'r','DisplayName','MinEstimatedObjective')
legend('MinObjective','MinEstimatedObjective')
xlabel('iteration')
ylabel('objective')
xlim([1 N_iter-N0])
figName=append(dir, idName, '_log.png');
% saveas(gcf,figName)

objectiveData_dir=append(dir, idName, '_objectiveData_not_removed.mat');
% save(objectiveData_dir,'objectiveData_not_removed');
objectiveEstData_dir=append(dir, idName, '_objectiveEstData_not_removed.mat');
% save(objectiveEstData_dir,'objectiveEstData_not_removed');
InitData_dir=append(dir, idName, '_InitData.mat');
% save(InitData_dir,'InitData');

pause;
close all;

end