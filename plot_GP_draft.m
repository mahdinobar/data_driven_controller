function plot_GP_draft()
clc
close all;
clear all; 
% % load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_68/results_1/trace_file_BO.mat')
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v4_0_10/trace_file_BO.mat")
% tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% idName= 'demo_GBO_v4_0_10';
% sys='DC_motor';
% dir=append(tmp_dir,'/', idName, '/');
% load(append(dir,'trace_file_BO.mat'))
% load(append(dir,'likelihood_dp.mat'))
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_8/grount_truth.mat")
% 
% % post_sigma2s=[];
% % post_mus=[];
% % values=[];
% % samples_P=[];
% % samples_I=[];
% % for i=1:2
% %     post_sigma2s=[post_sigma2s, Trace(i).post_sigma2s];
% %     post_mus=[post_mus, Trace(i).post_mus];
% %     values=[values, Trace(i).values];
% %     samples_P=[samples_P, Trace(i).samples(:,1)];
% %     samples_I=[samples_I, Trace(i).samples(:,2)];
% % end
% % 
% % fig=figure(1);
% % fig.Position=[200 0 1000 800];
% % ax1=axes;
% % ax1.FontSize=24;
% % ax1.FontName='Times New Roman';
% % hold on
% % y = mean(post_sigma2s,2); % your mean vector;
% % y=y(2:51)';
% % x = 1:50;
% % std_dev = var(post_sigma2s');
% % std_dev=std_dev(2:51);
% % curve1 = y + std_dev;
% % curve2 = y - std_dev;
% % x2 = [x, fliplr(x)];
% % inBetween = [curve1, fliplr(curve2)];
% % fill(ax1, x2, inBetween, 'g');
% % hold on; grid on;
% % plot(ax1, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax1, 'Iteration')
% % ylabel(ax1, '\sigma_{GP}^2')
% % ylim([0 5])
% % xlim([1 50])
% % title('GP uncertainty at sampled point')
% % 
% % fig=figure(11);
% % fig.Position=[200 0 1000 800];
% % ax11=axes;
% % ax11.FontSize=24;
% % ax11.FontName='Times New Roman';
% % hold on
% % exper=1;
% % y = sqrt(Trace(exper).post_sigma2s)./abs(Trace(exper).post_mus); % your mean vector;
% % y=y(2:51)';
% % x = 1:50;
% % % std_dev = var(post_sigma2s');
% % % std_dev=std_dev(2:51);
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax11, x2, inBetween, 'g');
% % hold on; grid on;
% % plot(ax11, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax11, 'Iteration')
% % ylabel(ax11, '\sigma_{GP}/|\mu_{GP}|')
% % ylim([0 1])
% % xlim([1 50])
% % title('GP model coefficient of variance in sampled point by aquisition function')
% % 
% % fig=figure(12);
% % fig.Position=[200 0 1000 800];
% % ax12=axes;
% % ax12.FontSize=24;
% % ax12.FontName='Times New Roman';
% % hold on
% % exper=1;
% % y = mean(sqrt(Trace(exper).post_sigma2s_record)./abs(Trace(exper).post_mus_record),1);
% % % y=y(2:51)';
% % x = 1:50;
% % % std_dev = var(post_sigma2s');
% % % std_dev=std_dev(2:51);
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax11, x2, inBetween, 'g');
% % hold on; grid on;
% % plot(ax12, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax12, 'Iteration')
% % ylabel(ax12, 'E(\sigma_{GP}/|\mu_{GP}|)')
% % ylim([0 1])
% % xlim([1 50])
% % title('GP model expected coefficient of variance in entire feasible hypergrid')
% % 
% % 
% % 
% % fig2=figure(2);
% % fig2.Position=[200 0 1000 800];
% % ax2=axes;
% % ax2.FontSize=24;
% % ax2.FontName='Times New Roman';
% % hold on
% % diff=post_mus-values;
% % y = mean(diff,2); % your mean vector;
% % y=y(2:51)';
% % x = 1:50;
% % std_dev = var(diff');
% % std_dev=std_dev(2:51);
% % curve1 = y + std_dev;
% % curve2 = y - std_dev;
% % x2 = [x, fliplr(x)];
% % inBetween = [curve1, fliplr(curve2)];
% % fill(ax2, x2, inBetween, 'g');
% % hold on; grid on;
% % plot(ax2, x, y, '-ko', 'LineWidth', 2);
% % ylim([-1 1])
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax2, 'Iteration')
% % ylabel(ax2, '\mu_{GP}-J')
% % title('GP uncertainty at sampled point')
% % ylim([-5 5])
% % xlim([1 50])
% % 
% % % fig=figure(3);
% % % fig.Position=[200 0 1000 800];
% % % ax3=axes;
% % % ax3.FontSize=24;
% % % ax3.FontName='Times New Roman';
% % % hold on
% % % exper=1;
% % % N0=1;
% % % [x,idx] = sort(Trace(exper).hyper_grid_record(:,1)');
% % % y=Trace(exper).post_mus_record(:,end)';
% % % y=y(idx);
% % % std_dev=Trace(exper).post_sigma2s_record(:,end)';
% % % std_dev=std_dev(idx);
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax3, x2, inBetween, 'y');
% % % hold on; grid on;
% % % plot(ax3, x, y, 'r', 'LineWidth', 2);
% % % % errorbar(ax3, x,y,std_dev)
% % % v=values(1+N0:end,exper);
% % % p=samples_P(1+N0:end,exper);
% % % scatter(p,v,'ok','filled')
% % % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % % xlabel(ax3, 'P gain')
% % % ylabel(ax3, 'latent value f(P,I)')
% % % title("GP model of BO after 50 Iterations")
% % % % ylim([-1 1])
% % % 
% % % fig=figure(4);
% % % fig.Position=[200 0 1000 800];
% % % ax4=axes;
% % % ax4.FontSize=24;
% % % ax4.FontName='Times New Roman';
% % % hold on
% % % exper=1;
% % % N0=1;
% % % [x,idx] = sort(Trace(exper).hyper_grid_record(:,2)');
% % % y=Trace(exper).post_mus_record(:,end)';
% % % y=y(idx);
% % % std_dev=Trace(exper).post_sigma2s_record(:,end)';
% % % std_dev=std_dev(idx);
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax4, x2, inBetween, 'y');
% % % hold on; grid on;
% % % plot(ax4, x, y, 'r', 'LineWidth', 2);
% % % v=values(1+N0:end,exper);
% % % p=samples_I(1+N0:end,exper);
% % % scatter(p,v,'ok','filled')
% % % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % % xlabel(ax4, 'I gain')
% % % ylabel(ax4, 'latent value f(P,I)')
% % % title("GP model of BO after 50 Iterations")
% % 
% % 
% % fig=figure(5);
% % fig.Position=[200 0 1000 800];
% % ax5=axes;
% % ax5.FontSize=24;
% % ax5.FontName='Times New Roman';
% % hold on;
% % grid on;
% % exper=2;
% % N0=1;
% % x=Trace(exper).hyper_grid_record(:,1);
% % y=Trace(exper).hyper_grid_record(:,2);
% % z=Trace(exper).post_mus_record(:,end);
% % xs=samples_P(1+N0:end,exper);
% % ys=samples_I(1+N0:end,exper);
% % zs=values(1+N0:end,exper);
% % % h1=surf(ax5, x,y,z);
% % dt = delaunayTriangulation(x,y) ;
% % tri = dt.ConnectivityList ;
% % xi = dt.Points(:,1) ; 
% % yi = dt.Points(:,2) ; 
% % F = scatteredInterpolant(x,y,z);
% % zi = F(xi,yi) ;
% % h1=trisurf(tri,xi,yi,zi,'FaceColor','black','FaceAlpha',0.3);
% % view(45,30)
% % % view(45,0)
% % % shading interp
% % h2=scatter3(xs,ys,zs,"or","filled", 'SizeData', 50);
% % h3=surf(kp_pt,ki_pt,reshape(j_pt,size(kp_pt)));
% % set(gca,'zscale','log')
% % set(gca,'ColorScale','log')
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax5, 'P gain')
% % ylabel(ax5, 'I gain')
% % zlabel(ax5, 'latent value f(P,I)')
% % title("GP model of BO after 50 Iterations")
% % legend([h1, h2, h3],{'GP model', 'BO samples', 'ground truth'}, 'Location', 'northeast'); 
% 
% fig=figure(6);
% fig.Position=[200 0 1000 800];
% ax6=axes;
% ax6.FontSize=24;
% ax6.FontName='Times New Roman';
% hold on;
% grid on;
% exper=1;
% hyp_GP_cov=Trace(exper).hyp_GP_cov;
% hyp_GP_lik=Trace(exper).hyp_GP_lik;
% % hyp_GP_mean=Trace(exper).hyp_GP_mean; %zeros(50,1);
% % h1=plot(ax6,hyp_GP_mean,'-o');
% h2=plot(ax6,hyp_GP_cov,'-o');
% h3=plot(ax6,hyp_GP_lik,'-o');
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% ylabel(ax6, 'GP hyperparameter')
% xlabel(ax6, 'iteration')
% xlim([1 50])
% ylim([-10 10])
% title("GP model Hyperparameters evolution")
% % legend([h2(1), h2(2),h2(3), h3],{'@covMaternard: ln(\lambda_1)', '@covMaternard: ln(\lambda_2)', '@covMaternard: ln(\sigma_f)', '@likGauss: ln(\sigma)'}, 'Location', 'best'); 
% legend([h2(1), h2(2), h3],{'@covMaternard: ln(l)', '@covMaternard: ln(\sigma_f)', '@likGauss: ln(\sigma)'}, 'Location', 'best'); 
% 
% % fig=figure(7);
% % fig.Position=[200 0 1000 800];
% % ax7=axes;
% % ax7.FontSize=24;
% % ax7.FontName='Times New Roman';
% % hold on;
% % grid on;
% % exper=1;
% % alpha=Trace(exper).GP_posterior_record.alpha;
% % sW=Trace(exper).GP_posterior_record.sW;
% % L=Trace(exper).GP_posterior_record.L;
% % h1=plot(ax7,alpha,'-o');
% % h2=plot(ax7,sW,'-o');
% % h3=plot(ax7,L,'-o');
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % ylabel(ax7, 'posterior')
% % xlabel(ax7, 'iteration')
% % xlim([1 50])
% % ylim([-100 100])
% % title("GP approximate posterior")
% % legend([h1, h2],{'alpha', 'sW'}, 'Location', 'best'); 
% % 
% % 
% % fig=figure(8);
% % fig.Position=[200 0 1000 800];
% % ax8=axes;
% % ax8.FontSize=24;
% % ax8.FontName='Times New Roman';
% % hold on
% % exper=2;
% % y = mean(Trace(exper).post_sigma2s_record,1); % your mean vector;
% % x = 1:50;
% % std_dev = var(Trace(1).post_sigma2s_record,1);
% % curve1 = y + std_dev;
% % curve2 = y - std_dev;
% % x2 = [x, fliplr(x)];
% % inBetween = [curve1, fliplr(curve2)];
% % fill(ax8, x2, inBetween, [.7 .7 .7]);
% % hold on; grid on;
% % plot(ax8, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax8, 'Iteration')
% % ylabel(ax8, '\sigma_{GP}^2')
% % ylim([0 10])
% % xlim([1 50])
% % xticks([1, 5:5:50])
% % title('GP uncertainty at entire hyper grid recorded')
% % 
% % 
% fig=figure(9);
% fig.Position=[200 0 1000 800];
% ax9=axes;
% ax9.FontSize=24;
% ax9.FontName='Times New Roman';
% hold on
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/J_truth_hyper_grid_record_1.mat")
% diff=Trace(exper).post_mus_record-j_pt;
% y = mean(diff); % your mean vector;
% x = 1:50;
% std_dev = sqrt(var(diff));
% curve1 = y + std_dev;
% curve2 = y - std_dev;
% x2 = [x, fliplr(x)];
% inBetween = [curve1, fliplr(curve2)];
% fill(ax9, x2, inBetween, [.7 .7 .7]);
% hold on; grid on;
% plot(ax9, x, y, '-ko', 'LineWidth', 2);
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax9, 'Iteration')
% ylabel(ax9, '\mu_{GP}-J')
% ylim([-5 5])
% xlim([1 50])
% xticks([1, 5:5:50])
% noise_level=0.0035; %mean(j_pt)*5/100;%(1.6957*5/100);
% h=yline(ax9,noise_level,'--')
% yline(ax9,-noise_level,'--')
% legend([h],'noise level')
% title('GP uncertainty at entire hyper grid recorded')
% 
% fig=figure(10);
% fig.Position=[200 0 1000 800];
% ax10=axes;
% ax10.FontSize=24;
% ax10.FontName='Times New Roman';
% hold on
% load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/J_truth_hyper_grid_record_1.mat")
% diff1=(Trace(exper).post_mus_record-j_pt);
% diff2=diff1.^2;
% y = mean(diff2); % your mean vector;
% x = 1:50;
% % std_dev = sqrt(var(diff2));
% % curve1 = y + std_dev;
% % curve2 = y - std_dev;
% % x2 = [x, fliplr(x)];
% % inBetween = [curve1, fliplr(curve2)];
% % fill(ax10, x2, inBetween, [.7 .7 .7]);
% hold on; grid on;
% plot(ax10, x, y, '-ko', 'LineWidth', 2);
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax10, 'Iteration')
% ylabel(ax10, 'MSE')
% ylim([0 5])
% xlim([1 50])
% xticks([1, 5:5:50])
% noise_level=0.0035; %mean(j_pt)*5/100;%(1.6957*5/100);
% h=yline(ax10,noise_level,'--')
% yline(ax10,-noise_level,'--')
% legend([h],'noise level')
% title('Mean Squared Error')
% 
% % fig=figure(14);
% % fig.Position=[200 0 1000 800];
% % ax14=axes;
% % ax14.FontSize=24;
% % ax14.FontName='Times New Roman';
% % hold on
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/J_truth_hyper_grid_record_1.mat")
% % tol=1e-1;
% % y=zeros(1,50);
% % for i=2:51
% %     s=Trace(exper).samples(i,:);
% %     d=Trace(exper).hyper_grid_record-s;
% %     idx=find(vecnorm(d,2,2)<tol);
% %     size(idx)
% %     diff2=(Trace(exper).post_mus_record(idx)-j_pt(idx)).^2;
% %     y(i-1)=mean(diff2);
% % end
% % x = 1:50;
% % % std_dev = sqrt(var(diff2));
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax14, x2, inBetween, [.7 .7 .7]);
% % hold on; grid on;
% % plot(ax14, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax14, 'Iteration')
% % ylabel(ax14, 'MSE')
% % ylim([0 5])
% % xlim([1 50])
% % xticks([1, 5:5:50])
% % noise_level=0.0035; %mean(j_pt)*5/100;%(1.6957*5/100);
% % h=yline(ax14,noise_level,'--')
% % yline(ax14,-noise_level,'--')
% % legend([h],'noise level')
% % title('Mean Squared Error (test points are around sampled data)')
% % 
% % 
% % fig=figure(13);
% % fig.Position=[200 0 1000 800];
% % ax13=axes;
% % ax13.FontSize=24;
% % ax13.FontName='Times New Roman';
% % hold on
% % load("/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_6/J_truth_hyper_grid_record_1.mat")
% % diff2=(Trace(exper).post_mus_record-j_pt).^2;
% % y = mean(diff2)./std(j_pt); % your mean vector;
% % x = 1:50;
% % % std_dev = sqrt(var(diff2));
% % % curve1 = y + std_dev;
% % % curve2 = y - std_dev;
% % % x2 = [x, fliplr(x)];
% % % inBetween = [curve1, fliplr(curve2)];
% % % fill(ax13, x2, inBetween, [.7 .7 .7]);
% % hold on; grid on;
% % plot(ax13, x, y, '-ko', 'LineWidth', 2);
% % set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% % xlabel(ax13, 'Iteration')
% % ylabel(ax13, 'SMSE')
% % ylim([0 5])
% % xlim([1 50])
% % xticks([1, 5:5:50])
% % noise_level=0.0035; %mean(j_pt)*5/100;%(1.6957*5/100);
% % h=yline(ax13,noise_level,'--')
% % yline(ax13,-noise_level,'--')
% % legend([h],'noise level')
% % title('Standardized Mean Squared Error')
% 
% fig=figure(12);
% fig.Position=[200 0 1000 800];
% ax12=axes;
% ax12.FontSize=24;
% ax12.FontName='Times New Roman';
% hold on
% zlimit=0.0035*3;
% x=Trace(1).hyper_grid_record(:,1);
% y=Trace(1).hyper_grid_record(:,2);
% z=diff2(:,end); %for last iteration
% h=scatter3(Trace(1).samples(2:end,1),Trace(1).samples(2:end,2),zlimit.*ones(50,1),'ro','filled')
% xv = linspace(min(x), max(x), 500);
% yv = linspace(min(y), max(y), 500);
% [X,Y] = meshgrid(xv, yv);
% Z = griddata(x,y,z,X,Y);
% surf(ax12, X, Y, Z);
% colorbar
% caxis([0 zlimit])
% set(gca,'zscale','log')
% % set(gca,'ColorScale','log')
% grid on
% set(gca, 'ZLim',[0 100])
% shading interp
% title('Squared Error in feasible set')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax12, 'kp')
% ylabel(ax12, 'ki')
% zlabel(ax12, 'SE')
% xlim([min(x), max(x)])
% ylim([min(y), max(y)])
% zlim([0 zlimit])
% [M,I]=min(j_pt);
% h2=scatter3(Trace(1).hyper_grid_record(I,1),Trace(1).hyper_grid_record(I,2),zlimit,72,'go','filled')
% legend([h, h2],'BO sampled gains','gt optimum')
% view(2)
% 
% fig=figure(13);
% fig.Position=[200 0 1000 800];
% ax13=axes;
% ax13.FontSize=24;
% ax13.FontName='Times New Roman';
% hold on
% zlimit=sqrt(0.0035)*3;
% x=Trace(1).hyper_grid_record(:,1);
% y=Trace(1).hyper_grid_record(:,2);
% z=diff1(:,end); %for last iteration
% h=scatter3(Trace(1).samples(2:end,1),Trace(1).samples(2:end,2),zlimit.*ones(50,1),'ro','filled')
% xv = linspace(min(x), max(x), 500);
% yv = linspace(min(y), max(y), 500);
% [X,Y] = meshgrid(xv, yv);
% Z = griddata(x,y,z,X,Y);
% surf(ax13, X, Y, Z);
% colorbar
% caxis([-zlimit zlimit])
% set(gca,'zscale','log')
% % set(gca,'ColorScale','log')
% grid on
% set(gca, 'ZLim',[0 100])
% shading interp
% title('Residual Error (f-y) in feasible set')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax13, 'kp')
% ylabel(ax13, 'ki')
% zlabel(ax13, 'SE')
% xlim([min(x), max(x)])
% ylim([min(y), max(y)])
% zlim([-zlimit zlimit])
% [M,I]=min(j_pt);
% h2=scatter3(Trace(1).hyper_grid_record(I,1),Trace(1).hyper_grid_record(I,2),zlimit,72,'go','filled')
% legend([h, h2],'BO sampled gains','gt optimum')
% view(2)
% 
% 
% fig=figure(15);
% fig.Position=[200 0 1000 800];
% ax15=axes;
% ax15.FontSize=24;
% ax15.FontName='Times New Roman';
% hold on
% x=cov_dp(:,2);
% y=cov_dp(:,3);
% z=-nlZ_dp;
% h=scatter3(x,y,z,10,'ro','filled');
% xv = linspace(min(x), max(x), 500);
% yv = linspace(min(y), max(y), 500);
% [X,Y] = meshgrid(xv, yv);
% Z = griddata(x,y,z,X,Y);
% % surf(ax15, X, Y, Z);
% contour(ax15,X,Y,Z,linspace(max(z)*0.1,max(z),5));
% colorbar
% % caxis([max(z)*0.8 max(z)])
% % set(gca,'zscale','log')
% % set(gca,'ColorScale','log')
% grid on
% set(gca, 'ZLim',[min(z)*0.95, max(z)*1.05])
% shading interp
% title('log marginal likelihood vs hyperparameters')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax15, '\lambda_{1}')
% ylabel(ax15, '\lambda_{2}')
% zlabel(ax15, 'log(L)')
% xlim([min(x), max(x)])
% ylim([min(y), max(y)])
% % zlim([min(z), max(z)])
% load(append(dir,'hyp.mat'),'hyp')
% load(append(dir,'fhyp.mat'),'fhyp')
% h2=scatter3(hyp.cov(2),hyp.cov(3),-fhyp(end),72,'go','filled');
% legend([h, h2],'LogL points','optimum LogL')
% view(2)

load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_72_tmp/y_s_1.mat')
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_72_tmp/trace_file.mat')
fig=figure(17);
fig.Position=[200 0 1000 800];
ax17=axes;
ax17.FontSize=24;
ax17.FontName='Times New Roman';
hold on
grid on
h1=plot(Trace(1).post_sigma2s(2:end));
N0=1;
sigma_s=[];
for i=1:50
    SE=0;
    for j=1:i
        SE=SE+(y_s(j)-Trace(1).values(j+N0))^2;
    end
    sigma_s=[sigma_s;sqrt(SE/i)];
end
h2=plot(sigma_s,'r');
title('Uncertainty vs iteration')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax17, 'iteration')
ylabel(ax17, '\sigma')
legend([h1, h2],'\sigma_{GP}','\sigma_{s}')
xlim([1, 50])
xticks([1, 5:5:50])

fig=figure(18);
fig.Position=[200 0 1000 800];
ax18=axes;
ax18.FontSize=24;
ax18.FontName='Times New Roman';
hold on
grid on
N0=1;
sigma_s=[];
for i=1:50
    SE=0;
    for j=1:i
        SE=SE+(y_s(j)-Trace(1).values(j+N0))^2;
    end
    sigma_s=[sigma_s;sqrt(SE/i)];
end
h1=plot(Trace(1).post_sigma2s(2:end)./sigma_s);
title('Uncertainty ratio vs iteration')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax18, 'iteration')
ylabel(ax18, '$\frac{\sigma_{GP}}{\sigma_{s}}$','Interpreter','latex')
xlim([1, 50])
xticks([1, 5:5:50])

fig=figure(19);
fig.Position=[200 0 1000 800];
ax19=axes;
ax19.FontSize=24;
ax19.FontName='Times New Roman';
hold on
grid on
N0=1;
sigma_s=[];
for i=1:50
    SE=0;
    for j=1:i
        SE=SE+(y_s(j)-Trace(1).values(j+N0))^2;
    end
    sigma_s=[sigma_s;sqrt(SE/i)];
end
h1=plot(sigma_s,Trace(1).post_sigma2s(2:end));
title('Uncertainty ratio vs iteration')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax19, '\sigma_{s}')
ylabel(ax19, '\sigma_{GP}')


fig=figure(20);
fig.Position=[200 0 1000 800];
ax20=axes;
ax20.FontSize=24;
ax20.FontName='Times New Roman';
hold on
grid on
% eta1=[0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2];
% eta1_str={'05','06','07','08','09','1','11','12','13','14','15','16','17','18','19','2'};
% eta1=[1:10];
% eta1_str={'1','2','3','4','5','6','7','8','9','10'};
eta1=[0.5,1,1.5,2,3,5,10];
eta1_str={'05','1','15','2','3','5','10'};
mean_n_s=[];
std_n_s=[];
for i=1:length(eta1)
    n_s{i}=[];
%     load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_74_sigma_s_eta2_02_eta1_',eta1_str{i},'/results_1/trace_file.mat'))
% load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_72_eta2_02_eta1_',eta1_str{i},'/results_1/trace_file.mat'))
%     load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_75_eta2_02_eta1_',eta1_str{i},'/results_1/trace_file.mat'))
load(append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/server_data/GBO_76_eta2_02_eta1_',eta1_str{i},'/results_1/trace_file.mat'))
% dirGBO="/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/demo_GBO_v5_0_12/";
% load(append(dirGBO,'trace_file.mat'),'Trace')
    
for j=1:length(Trace)
    try
        %             n_s{i}=[n_s{i};size(Trace(j).G2_values,1)];
        n_s{i}=[n_s{i};size(Trace(j).hyp_GP_mean,1)-50];
    catch
        n_s{i}=[n_s{i};nan];
    end
end
    mean_n_s=[mean_n_s,nanmean(n_s{i})];
    std_n_s=[std_n_s,nanstd(n_s{i})];
end
clearvars Trace
yyaxis right
hr=errorbar(eta1,mean_n_s,std_n_s,'-or');
% hr=plot(eta1,mean_n_s,'-or');
title('Generalization of surrogate switch $\frac{\sigma_{GP}}{\sigma_{s}}>\eta_{1}$','Interpreter','latex')
% title('Generalization of surrogate switch $\sigma_{GP}>\eta_{1}$','Interpreter','latex')
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
xlabel(ax20, '\eta_{1}')
ylabel(ax20, 'total number of switching')
xticks(eta1)
ylim([0 max(mean_n_s+std_n_s)])
yyaxis left

% % server 72 results
% mean_GBO_convergance=[1.68000000000000	2.28000000000000	1.54000000000000	2.28000000000000	2.36000000000000	1.94000000000000	1.78000000000000	2.62000000000000	3.54000000000000	4.16000000000000];%[1     1     1     2     2     2     2     2     3     4];
% std_GBO_convergance=[1.55760150519303	2.83592118712630	1.34331299253687	2.83592118712630	2.61674357528121	2.05446253957832	1.98247423227457	3.42195790715890	4.14142832503818	5.61106778008010];

% % server 74 results
% mean_GBO_convergance=[2.82978723404255	3.53191489361702	4.83673469387755	3.82000000000000	3.61702127659575	4.16666666666667	3.87500000000000	3.08163265306122	2.77551020408163	4.37500000000000	4.65306122448980	4.75510204081633	5.12765957446809	5.61702127659575	4.62500000000000	4.87500000000000];
% std_GBO_convergance=[4.10894649005002	5.04264882011768	7.04848369856805	5.59405661284860	5.72391319279633	5.74023994358503	5.62526595115970	4.30521357723148	4.60645066782464	7.08797303198366	6.30327633195680	6.25010203998335	7.36801849734671	7.78391091901684	6.88391588213717	6.80933339443706];

% % server 75 results 
mean_GBO_convergance=[32.7142857142857	29.6875000000000	29.2352941176471	26.9473684210526	25.6250000000000	27.8181818181818	28.5555555555556];
std_GBO_convergance=[12.3315016954925	10.1174354458034	9.32422524773980	11.8812348975205	12.1976773745387	10.0741837132930	10.1007991659497];
% hl=plot(eta1,mean_GBO_convergance,'-ob');
hr=errorbar(eta1,mean_GBO_convergance,std_GBO_convergance,'-ob');
ylabel(ax20, 'Convergance iteration to ground truth performance')
% ylabel(ax20, 'Convergance iteration to nominal performance')
ylim([0 max(mean_GBO_convergance+std_GBO_convergance)])

% fig=figure(21);
% fig.Position=[200 0 1000 800];
% ax21=axes;
% ax21.FontSize=24;
% ax21.FontName='Times New Roman';
% hold on
% grid on
% 
% 
% title('Number of swtiching vs \eta_{1}')
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax21, '\eta_{1}')
% ylabel(ax21, 'total number of switching')

% fig=figure(16);
% fig.Position=[200 0 1000 800];
% contour(X,Y,Z,linspace(max(z)*0.1,max(z),5));
% hold on; grid on;
% scatter3(hyp.cov(2),hyp.cov(3),-fhyp(end),72,'go','filled');
% scatter3(x,y,z,10,'ro','filled');

% 
% fig=figure(10);
% fig.Position=[200 0 1000 800];
% ax10=axes;
% ax10.FontSize=24;
% ax10.FontName='Times New Roman';
% hold on
% grid on
% ratio=[];
% exper=2;
% for i=1:50
%     aq_m=Trace(exper).AQ_vals(i);
%     aq_M=max(Trace(exper).AQ_vals(1:i));
%     ratio(end+1)=aq_m/aq_M;
% end
% plot(ax10, ratio, '-ko', 'LineWidth', 2);
% set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% xlabel(ax10, 'Iteration')
% ylabel(ax10, 'EI_{m} / max EI_{m}')
% % ylim([-10 10])
% xlim([1 50])
% xticks([1, 5:5:50])
% title('EI over maximum EI vs iteration')

end
