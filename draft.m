function draft()
clc
close all;
clear all; 
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/trace_file_BO.mat')
post_sigma2s=[];
for i=1:100
    post_sigma2s=[post_sigma2s, Trace(i).post_sigma2s];
end

y = mean(post_sigma2s,2); % your mean vector;
y=y(2:51)';
x = 1:50;
std_dev = var(post_sigma2s');
std_dev=std_dev(2:51);
curve1 = y + std_dev;
curve2 = y - std_dev;
x2 = [x, fliplr(x)];
inBetween = [curve1, fliplr(curve2)];
fill(x2, inBetween, 'g');
hold on; grid on;
plot(x, y, 'r', 'LineWidth', 2);


end

