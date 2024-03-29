function rup_kel_2
clear all; clc; close all;
tmp_dir='/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp';
% hyper-params
idName= 'demo_GBO_0_2';
sys='DC_motor';
N0=3;
N_iter=30+10;
repeat_experiment=10;
withSurrogate=true;
N_real_repeat=5;
Nsample=50;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

dir=append(tmp_dir,'/', idName, '/');
if not(isfolder(dir))
    mkdir(dir)
end

%%
% % KUKA LBR IIWA
% Jl=5.6;
% bc=55;
% Jc=1.03;
% K=18500;
% Td=5;
% G = tf([Jl*bc, Jl*K],[Jc*Jl, Jl*bc, Jc*K+Jl*K],'InputDelay',Td);

% % ball-screw system
% Kcp=60;
% Kci=1000;
% Kcd=18;
% Ra=9.02;
% La=0.0187;
% Kt=0.515;
% Kb=0.55;
% Jm=0.27e-4;
% Bm=0.0074;
% Jl=6.53e-4;
% Bml=0.014;
% Ks=3e7;
% G=tf([Kt],[La*(Jm+Jl),La*Bm+Ra*(Jm+Jl),Ra*Bm+Kt*Kb])*tf([Bml,Ks],[Jl,Bml,Ks]);

% % robot-arm System ("Simultaneous computation of model order..., Badaruddin Muhammad et al.")
% num = [-0.0118, 0.0257, 0, 0, 0];
% den = [1, -3.1016, 4.3638, -3.1528, 1.0899, -0.0743];
% ts=1;
% G = d2c(tf(num,den, ts));

% % DC motor at FHNW lab
% num = [5.19908];
% den = [1, 1.61335];
% Td=2e-3;
% % MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
% G = tf(num, den, 'InputDelay',Td);
% DC motor at FHNW lab (new)
% speed sensor pole 0.003
% num = [195.199];
% den = [1, 32.6931, 60.7286];
% speed sensor pole 9.918e-5
num = [9.54434];
den = [1, 4.14479, 4.19941];
Td=2e-3;
% MATLAB: "For SISO transfer functions, a delay at the input is equivalent to a delay at the output. Therefore, the following command creates the same transfer function:"
G = tf(num, den, 'InputDelay',Td);
[k,ku,tu]=znpidtuning(G,2)
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds/KpKi_bounds.mat')
% Kp_nom=0.55;
% Ki_nom=1.5;
% Ctl_nom_1=tf([Kp_nom,Kp_nom*Ki_nom], [1, 0]);
% [GmUnit,Pm,Wcg,Wcp] = margin(Ctl_nom_1*G)
% GmdB=20*log10(GmUnit)
% J_nominal=-funPS([Kp_nom, Ki_nom], G)
% true_objective = 4.1000;
% ORnom=J_nominal/true_objective
%%
%
% % uncomment to estimate stable gain bounds
% % auto tune
% [C_tuned,info] = pidtune(G,'PI');
% % Kd_nominal=C_tuned.Kd;
% Kp_nominal_1=C_tuned.Kp;
% Ki_nominal_1=C_tuned.Ki;

% phase gain margin nominal controller: WENG KHUEN HO et al.
% taw=0.505;
% L=2/1000;

% % speed sensor pole 0.003
% kp=3.214;

% speed sensor pole 9.918e-5
kp=9.54434*(1.76351*2.38129);


taw1=tu/(2*pi)*sqrt(ku*kp-1);
L1=tu/(2*pi)*(pi-2*atan(2*pi*taw1/tu));
t1=1.24*taw1+L1;
t2=3.40*taw1+L1;
taw=0.67*(t2-t1);
L=1.3*t1-0.29*t2;
Am_db=10; %gain margin in dB
Am=10^(Am_db/20);
Pm=60*pi/180;
Wp=(Am*Pm+.5*pi*Am*(Am-1))/((Am^2-1)*L)
Kp_nominal=(Wp*taw)/(Am*kp)
Ki_nominal=2*Wp-4*Wp^2*L/pi+1/taw

% % validate approximated phase gain margins
% num = [kp];
% den = [taw, 1];
% Td=L;
% Gtmp = tf(num, den, 'InputDelay',Td);
K=tf([Kp_nominal, Kp_nominal*Ki_nominal], [1, 0]);
% K=tf([0.6, 0.6*1.2], [1, 0]);

[K,info] = pidtune(G,'PI')


allmargin(K*G)
margin(K*G)
figure(2)
step(feedback(K,G,-1))
% figure(2)
% step(feedback(K*G,1,-1))
%  calculate gain feasible set
max_overshoot=0;
d=1e-1;
while max_overshoot<50 && ~isnan(max_overshoot)
    lb=[Kp_nominal-d, Ki_nominal-d];
    ub=[Kp_nominal+d, Ki_nominal+d];
    funPS_handle = @(x)funOv(x, G);
    x = particleswarm(funPS_handle,2,lb,ub);

    Ctmp=tf([x(1), x(1)*x(2)], [1, 0]);
    CLtmp=feedback(Ctmp*G, 1);
    d
    max_overshoot=stepinfo(CLtmp).Overshoot
    d = d*1.5;
end
d=d/1.5;

max_overshoot=0;
di=1e-1;
while max_overshoot<100 && ~isnan(max_overshoot)
    lb=[Kp_nominal-d, Ki_nominal-d-di];
    ub=[Kp_nominal+d, Ki_nominal+d+di];
    funOV_handle = @(x)funOv(x, G);
    x = particleswarm(funOV_handle,2,lb,ub);

    Ctmp=tf([x(1), x(1)*x(2)], [1, 0]);
    CLtmp=feedback(Ctmp*G, 1);
    di
    max_overshoot=stepinfo(CLtmp).Overshoot
    di = di*1.1;
end
di=di/1.1;

max_overshoot=0;
dp=1e-1;
while max_overshoot<75 && ~isnan(max_overshoot)
    lb=[Kp_nominal-d-dp, Ki_nominal-d-di];
    ub=[Kp_nominal+d+dp, Ki_nominal+d+di];
    funOV_handle = @(x)funOv(x, G);
    x = particleswarm(funOV_handle,2,lb,ub);

    Ctmp=tf([x(1), x(1)*x(2)], [1, 0]);
    CLtmp=feedback(Ctmp*G, 1);
    dp
    max_overshoot=stepinfo(CLtmp).Overshoot
    dp = dp*1.1;
end
    function [objective] = funOv(x, G)
%             todo move some lines outside with handler@: faster?
        CC=tf([x(1), x(1)*x(2)], [1, 0]);
        CCL=feedback(CC*G, 1);

        objective=-abs(stepinfo(CCL).Overshoot);
        if isnan(objective)
            objective=-1e10;
        end
    end

dp=dp/1.1;

Kp_min=Kp_nominal-d-dp;
Kp_max=Kp_nominal+d+dp;
Ki_min=Ki_nominal-d-di;
Ki_max=Ki_nominal+d+di;
save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds/KpKi_bounds_new_2.mat','Kp_min','Ki_min','Kp_max', 'Ki_max')

J_nominal=-funPS([Kp_nominal, Ki_nominal], G)
true_objective = 4.1000;
ORnom=J_nominal/true_objective
%
% Kp_nominal=0.6119;
% Ki_nominal=1.6560;





Ki_min=1.27554816862556;
Ki_max=1.84617150984756;
Kp_min=0.0672261578128495;
Kp_max=0.585974649832850;

Ctl_pgm=tf([50.32,50.32*1.61335], [1, 0]);
CL_pgm=feedback(Ctl_pgm*G, 1);
Ctl_nom_1=tf([Kp_nominal_1,Kp_nominal_1*Ki_nominal_1], [1, 0]);
Ctl_nom=tf([Kp_nominal,Kp_nominal*Ki_nominal], [1, 0]);
CL_nom=feedback(Ctl_nom*G, 1);
Ctl_max=tf([Kp_max,Kp_max*Ki_max], [1, 0]);
CL_max=feedback(Ctl_max*G, 1);
Ctl_min=tf([Kp_min,Kp_min*Ki_min], [1, 0]);
CL_min=feedback(Ctl_min*G, 1);
[Gm,Pm,Wcg,Wcp] = margin(Ctl_nom_1*G)

% step(CL_pgm);hold on; step(CL_nom, 'k');hold on; step(CL_max,'r');hold on; step(CL_min,'r');


%% plot step response per various benchmark tunings
fig=figure();
fig.Position=[200 0 1600 800];
ax1=axes;
ax1.FontSize=24;
ax1.FontName='Times New Roman';
hold on

Tf=4;
resol=2000;
dt=Tf/resol;
x0=0;
time=0:dt:Tf;
u_ref1=1.*ones(floor(size(time,2)/2),1);
u_ref2=0.*ones(ceil(size(time,2)/2),1);
u_ref=[u_ref1;u_ref2];

Kp_gt=0.6119;
Ki_gt=1.6636;
Ctl_gt=tf([Kp_gt,Kp_gt*Ki_gt], [1, 0]);
CL_gt=feedback(Ctl_gt*G, 1);
Kp_nominal=0.4873;
Ki_nominal=1.5970;
Ctl_nom=tf([Kp_nominal,Kp_nominal*Ki_nominal], [1, 0]);
CL_nom=feedback(Ctl_nom*G, 1);
Kp_GBO_best=0.5605;
Ki_GBO_best=1.5432;
Ctl_GBO_best=tf([Kp_GBO_best,Kp_GBO_best*Ki_GBO_best], [1, 0]);
CL_GBO_best=feedback(Ctl_GBO_best*G, 1);
Kp_GBO_worst=0.4369;
Ki_GBO_worst=1.6667;
Ctl_GBO_worst=tf([Kp_GBO_worst,Kp_GBO_worst*Ki_GBO_worst], [1, 0]);
CL_GBO_worst=feedback(Ctl_GBO_worst*G, 1);
Kp_BO=0.5697;
Ki_BO=1.5057;
Ctl_BO=tf([Kp_BO,Kp_BO*Ki_BO], [1, 0]);
CL_BO=feedback(Ctl_BO*G, 1);
y_gt = lsim(CL_gt, u_ref, time, x0, 'zoh');
y_nom = lsim(CL_nom, u_ref, time, x0, 'zoh');
y_GBO_best = lsim(CL_GBO_best, u_ref, time, x0, 'zoh');
y_GBO_worst = lsim(CL_GBO_worst, u_ref, time, x0, 'zoh');
y_BO = lsim(CL_BO, u_ref, time, x0, 'zoh');

h1=plot(time, 245.31+(306.64-245.31)*y_gt, 'g', 'LineWidth', 3);
h2=plot(time, 245.31+(306.64-245.31)*y_nom, 'k', 'LineWidth', 3);
h3=plot(time, 245.31+(306.64-245.31)*y_GBO_best, 'r', 'LineWidth', 3);
h4=plot(time, 245.31+(306.64-245.31)*y_GBO_worst, 	'Color', 'b', 'LineWidth', 3);
% h5=plot(time, y_BO, 'b', 'LineWidth', 3);
h5=stairs([0,Tf/2,Tf], [306,245,245],'--k', 'LineWidth', 3);

legend([h1, h2, h3, h4, h5],{'Ground Truth', 'Nominal PGM', 'Guided BO', 'BO', 'Reference Input'}, 'Location', 'northeast');
grid on
xlim(ax1, [0 Tf])
yticks([245, 306])
xlabel(ax1, 'Time (s)')
ylabel(ax1, 'Speed (rad/s)')
% ax1.title(append('Optimality Ratio vs Iteration (N0=',num2str(N0),')'))
set(gca, 'DefaultAxesFontName', 'Times New Roman', 'FontSize', 24)
% set(gca,'yscale','log')
figName=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/','StepRsps.png');
saveas(gcf,figName)
figName=append('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/','StepRsps.fig');
saveas(gcf,figName)

%%
% Tf=1e-8;

% C=tf([Kd_nominal+Tf*Kp_nominal,Kp_nominal+Tf*Ki_nominal,Ki_nominal], [Tf, 1, 0]);
% CL=feedback(C*G, 1);
% if abs(stepinfo(CL).Overshoot)<1
%     objective = abs(1*stepinfo(CL).SettlingTime);
% else
%     objective = abs(stepinfo(CL).Overshoot*stepinfo(CL).SettlingTime);
% end
% objective
limit_objective=0;
d=1e-1;
while limit_objective<100 && ~isnan(limit_objective)
    d
    %     lb=[Kp_nominal-d, Ki_nominal-d];
    %     ub=[Kp_nominal+d, Ki_nominal+d];
    lb=[Kp_nominal-d, Ki_nominal-d];
    ub=[Kp_nominal+d, Ki_nominal+d];
    funPS_handle = @(x)funPS(x, G);
    x_ps = particleswarm(funPS_handle,2,lb,ub);

    Ctmp=tf([x_ps(1), x_ps(1)*x_ps(2)], [1, 0]);
    CLtmp=feedback(Ctmp*G, 1);
    %     max_overshoot=stepinfo(CLtmp).Overshoot
    limit_objective = -funPS(x_ps, G)

    d = d*1.1;
end
%     function [objective] = funPS(x, G)
%         %     todo move some lines outside with handler@: faster?
%         C=tf([x(1), x(1)*x(2)], [1, 0]);
%         CL=feedback(C*G, 1);
%         objective=-abs(stepinfo(CL).Overshoot);
%         if isnan(objective)
%             objective=-1e10;
%         end
%     end
    function [objective] = funPS(X, G)
        %     todo move some lines outside with handler@: faster?
        C=tf([X(1), X(1)*X(2)], [1, 0]);
        CL=feedback(C*G, 1);

        ov=abs(stepinfo(CL).Overshoot);
        st=stepinfo(CL).SettlingTime;

        [y,t]=step(CL);
        reference=1;
        e=abs(y-reference);
        Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.6]).RiseTime;
        ITAE = trapz(t, t.*abs(e));

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
        w=[0.1, 1, 1, 0.5];
%         w=[91.35, 0.34, 0.028, 0.0019];
% w=[40.	0.10	0.01	0.0002];
% w=[1, 1, 1, 1];
        w=w./sum(w);
        objective=-abs(ov/w(1)+st/w(2)+Tr/w(3)+ITAE/w(4));
        data_tmp=[ov/w(1), st/w(2), Tr/w(3), ITAE/w(4)];
        %         objective=-abs(abs(ov)/w1+st/w2+Tr/w3+ITAE/w4);
    end
d=d/1.1;
Kp_min=Kp_nominal-d
Kp_max=Kp_nominal+d
Ki_min=Ki_nominal-d
Ki_max=Ki_nominal+d
% Ki_min=1.27554816862556;
% Ki_max=1.84617150984756;
save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds/KpKi_bounds_GM3_PM60.mat','Kp_min','Ki_min', 'Kp_max','Ki_max')

% Kp_min=0.0672261578128495;
% Kp_max=0.585974649832850;

Ctl_pgm=tf([50.32,50.32*1.61335], [1, 0]);
CL_pgm=feedback(Ctl_pgm*G, 1);
Ctl_nom=tf([Kp_nominal,Kp_nominal*Ki_nominal], [1, 0]);
CL_nom=feedback(Ctl_nom*G, 1);
Ctl_max=tf([Kp_max,Kp_max*Ki_max], [1, 0]);
CL_max=feedback(Ctl_max*G, 1);
Ctl_min=tf([Kp_min,Kp_min*Ki_min], [1, 0]);
CL_min=feedback(Ctl_min*G, 1);
step(CL_pgm);hold on; step(CL_nom, 'k');hold on; step(CL_max,'r');hold on; step(CL_min,'r');


% % max_overshoot=0;
% % dp=1e-1;
% % di=1e-6;
% % dd=1e-6;
% % while max_overshoot<25 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     dp
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     dp = dp*1.5;
% % end
% % dp = dp/1.5;
% % max_overshoot=0;
% % while max_overshoot<50 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     di
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     di = di*1.5;
% % end
% % di = di/1.5;
% % max_overshoot=0;
% % while max_overshoot<75 && ~isnan(max_overshoot)
% %     lb=[Kp_nominal-dp, Ki_nominal-di, Kd_nominal-dd];
% %     ub=[Kp_nominal+dp, Ki_nominal+di, Kd_nominal+dd];
% %     funPS_handle = @(x)funPS(x, G, Tf);
% %     x = particleswarm(funPS_handle,3,lb,ub);
% %
% %     Ctmp=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %     CLtmp=feedback(Ctmp*G, 1);
% %     dd
% %     max_overshoot=stepinfo(CLtmp).Overshoot
% %     dd = dd*1.5;
% % end
% % dd = dd/1.5;
% %     function [objective] = funPS(x, G, Tf)
% %         %     todo move some lines outside with handler@: faster?
% %         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
% %         CL=feedback(C*G, 1);
% %         objective=-abs(stepinfo(CL).Overshoot);
% %         if isnan(objective)
% %             objective=-1e10;
% %         end
% %     end
% %
% % Kp_min=Kp_nominal-dp;
% % Kp_max=Kp_nominal+dp;
% % Ki_min=Ki_nominal-di;
% % Ki_max=Ki_nominal+di;
% % Kd_min=Kd_nominal-dd;
% % Kd_max=Kd_nominal+dd;
% % save('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/robot_arm_gain_bounds/KpKiKd_bounds.mat','Kp_min','Ki_min','Kd_min', 'Kp_max','Ki_max','Kd_max')
save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/tmp/DC_motor_gain_bounds/KpKi_bounds_b.mat','Kp_min','Ki_min', 'Kp_max','Ki_max')
%%
% if sys=="ball_screw"
%     dir_gains=append(tmp_dir,'/', 'ball_screw_gain_bounds', '/', 'KpKiKd_bounds.mat');
% elseif sys=="robot_arm"
%     dir_gains=append(tmp_dir,'/', 'robot_arm_gain_bounds', '/', 'KpKiKd_bounds.mat');
% elseif sys=="DC_motor"
%     dir_gains=append(tmp_dir,'/', 'DC_motor_gain_bounds', '/', 'KpKi_bounds.mat');
% end
% load(dir_gains)
%%
% % % Find True values
% % lb=[-0.0954016552295164*1.01,0.186083733952419*0.99,-0.0954016552295164*1.01];
% % ub=[-0.0954016552295164*0.99,0.186083733952419*1.01,-0.0954016552295164*0.99];

% lb=[-0.2358, 0.0457, -0.2358];
% ub=[0.2358, 0.5173, 0.2358];
% lb=[-0.0186267176164606, 1.21563271779725];
% ub=[0.671827525262160, 1.90608696067587];
lb = [0.0412887332118495, 1.27554816862556];
ub = [0.611912074433850, 1.84617150984756];

funPS_handle = @(x)funPS2(x, G);
% options = optimoptions('particleswarm','FunctionTolerance', 0, 'MaxIterations', 1e3, 'MaxStallIterations', 1e2, 'ObjectiveLimit', 0);
true_optimum_vars = particleswarm(funPS_handle,2,lb,ub)
% % x.Kp=0.231167276750434;
% % x.Ki=0.467605534040020;
% % x.Kd=0.234584924311749;
True_objective = funPS2(true_optimum_vars, G)
%     function [objective] = funPS2(x, G, Tf)
%         %         todo move some lines outside with handler@: faster?
%         C=tf([x(3)+Tf*x(1),x(1)+Tf*x(2),x(2)], [Tf, 1, 0]);
%         CL=feedback(C*G, 1);
%         ov=abs(stepinfo(CL).Overshoot);
%         st=stepinfo(CL).SettlingTime;
%         if isnan(ov) || isinf(ov) || ov>1e3
%             ov=1e3;
%         end
%         if isnan(st) || isinf(st) || st>1e5
%             st=1e5;
%         end
%
%         w1=1;     w2=500;     objective=ov/w1+st/w2;
%     end

    function [objective] = funPS2(x, G)
        %     todo move some lines outside with handler@: faster?
        C=tf([x(1),x(1)*x(2)], [1, 0]);
        CL=feedback(C*G, 1);

        ov=abs(stepinfo(CL).Overshoot);
        st=stepinfo(CL).SettlingTime;

        [y,t]=step(CL);
        reference=1;
        e=abs(y-reference);
        Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,0.98]).RiseTime;
        ITAE = trapz(t, t.*abs(e));

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

        w=[0.1, 1, 1, 0.5];
        w=w./sum(w);
        objective=ov/w(1)+st/w(2)+Tr/w(3)+ITAE/w(4);
    end



% Kpc=1.;
% Kic=100.;
% Kdc=0.;
% % search_span_d=100;
% search_span_p=2.;
% search_span_i=20.;
% search_span_d=.2;
% Kp_min=Kpc-search_span_p/2;
% Kp_max=Kpc+search_span_p/2;
% Ki_min=Kic-search_span_i/2;
% Ki_max=Kic+search_span_i/2;
% Kd_min=Kdc-search_span_d/2;
% Kd_max=Kdc+search_span_d/2;


% % add extra safety margin
% safeFacp=1e-1;
% safeFaci=1e-1;
% safeFacd=1e-1;
% rgKp=(Kp_max-Kp_min);
% Kp_min=Kp_min+safeFacp*rgKp;
% Kp_max=Kp_max-safeFacp*rgKp;
% rgKi=(Ki_max-Ki_min);
% Ki_min=Ki_min+safeFaci*rgKi;
% Ki_max=Ki_max-safeFaci*rgKi;
% rgKd=(Kd_max-Kd_min);
% Kd_min=Kd_min+safeFacd*rgKd;
% Kd_max=Kd_max-safeFacd*rgKd;

%%
tmp=[];
% % initial values for GP of BO
RAND=rand(N0,1);

% load(append(dir,'RAND.mat'))

Kp = (Kp_max-Kp_min).*RAND + Kp_min;
Ki = (Ki_max-Ki_min).*RAND + Ki_min;
InitobjectiveData = zeros(N0,1);
% todo pay attention how you choose sampleTf?
sampleTf=1.5;
sampleTs=sampleTf/(Nsample-1);
global data
for i=1:N0
    C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
    CL=feedback(C*G, 1);
    varstmp.Kp=Kp(i);
    varstmp.Ki=Ki(i);
    InitobjectiveData(i) = myObjfun_Loop(varstmp, G);
    while isnan(InitobjectiveData(i)) || InitobjectiveData(i)>1000
        RAND(i)=rand(1,1);
        Kp(i) = (Kp_max-Kp_min).*RAND(i) + Kp_min;
        Ki(i) = (Ki_max-Ki_min).*RAND(i) + Ki_min;
        C=tf([Kp(i), Kp(i)*Ki(i)], [1, 0]);
        CL=feedback(C*G, 1);
        varstmp.Kp=Kp(i);
        varstmp.Ki=Ki(i);
        InitobjectiveData(i) = myObjfun_Loop(varstmp, G);
    end
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    %         todo check concept?
    if i==1
        data = iddata(ytmp,utmp,sampleTs);
    else
        data = merge(data, iddata(ytmp,utmp,sampleTs));
    end
end
tmp=[tmp,RAND];
save(append(dir,'RAND_N01.mat'),'tmp')

%%
% surrogate model
% G2tmp = n4sid(data,np2);
% [A,B,C,D,~] = idssdata(G2tmp);
% [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
% G2 = tf(num_surrogate, den_surrogate);

G2 = tfest(data,np2);
% G2_2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.8e0, 0].*(rand(1,1)-0.5));
% G2_3=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
% G2_4=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
% G2_5=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
% G2_6=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
% step(G2, G2_2, G2_3, G2_4, G2_5, G2_6)
InitData=table(Kp, Ki);

%%
Kp = optimizableVariable('Kp', [Kp_min Kp_max], 'Type','real');
Ki = optimizableVariable('Ki', [Ki_min Ki_max], 'Type','real');

vars=[Kp, Ki];
if withSurrogate==true
    if withPerturbed==true
        fun = @(vars)myObjfun_ApproxLoop_perturbed(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat);
    else
        fun = @(vars)myObjfun_ApproxLoop(vars, G, G2, sampleTf, sampleTs, np2, N_real_repeat);
    end
else
    fun = @(vars)myObjfun_Loop(vars, G);
end
objectiveEstData=InitobjectiveData;
XobjectiveEstData=InitData;
objectiveData=InitobjectiveData;
XobjectiveData=InitData;
global N
global idx
N_iter=N_iter+N0;
data_start=data;
InitData_start=InitData;
InitobjectiveData_start=InitobjectiveData;
objectiveEstData_start=objectiveEstData;
XobjectiveEstData_start=XobjectiveEstData;
objectiveData_start=objectiveData;
XobjectiveData_start=XobjectiveData;
InitData_all=InitData;
InitobjectiveData_all=InitobjectiveData;
objectiveEstData_all=objectiveEstData;
XobjectiveEstData_all=XobjectiveEstData;
objectiveData_all=objectiveData;
XobjectiveData_all=XobjectiveData;
for experiment=1:repeat_experiment
    experiment

    N=[];
    idx=[];
    counter=N0+1;
    %     objectiveData_not_removed=InitobjectiveData;
    %     objectiveEstData_not_removed=objectiveEstData;
    for iter=N0+1:N_iter
        iter
        %     iteration=iter-N0
        nanCheck=nan;
        %         while isnan(nanCheck)
        results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', counter, 'NumSeedPoints', counter-1, ...
            'PlotFcn', {}, 'InitialObjective', InitobjectiveData, 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound');
        %             nanCheck = results.MinObjective;
        %             error('nanCheck is NAN');
        %         end
        InitData=[InitData; results.NextPoint];
        InitobjectiveData = [InitobjectiveData; myObjfun_Loop(results.NextPoint, G)];
        objectiveEstData = [objectiveEstData; results.MinEstimatedObjective];
        XobjectiveEstData = [XobjectiveEstData; results.XAtMinEstimatedObjective];
        objectiveData = [objectiveData; results.MinObjective];
        XobjectiveData = [XobjectiveData; results.XAtMinObjective];

        %         objectiveData_not_removed=[objectiveData_not_removed; results.MinObjective];
        %         objectiveEstData_not_removed=[objectiveEstData_not_removed; results.MinEstimatedObjective];

        %         remove previos data of older surrogate model
        if withSurrogate==true
            if withPerturbed==true
                %     uncomment for surrogate model
                %     remove previos data of older surrogate model
                if rem(iter-N0,N_real_repeat+(1+num_perturbed_model))==0 && iter>N0+N_real_repeat+(num_perturbed_model)
                    InitData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    InitobjectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    objectiveEstData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    objectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    XobjectiveEstData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    XobjectiveData([counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1],:)=[];
                    counter-N_real_repeat-(num_perturbed_model+1)+1:counter-N_real_repeat-(0+1)+1
                    counter=counter-(num_perturbed_model+1);
                end
            else
                if rem(iter-N0-1,N_real_repeat+1)==0 && iter>N0+1
                    InitData([counter-N_real_repeat-1],:)=[];
                    InitobjectiveData([counter-N_real_repeat-1],:)=[];
                    objectiveEstData([counter-N_real_repeat-1],:)=[];
                    objectiveData([counter-N_real_repeat-1],:)=[];
                    XobjectiveEstData([counter-N_real_repeat-1],:)=[];
                    XobjectiveData([counter-N_real_repeat-1],:)=[];
                    counter=counter-1;
                end
            end
        end
        counter=counter+1;
        %     FileName='results.mat';
        %     results = bayesopt(fun,vars, 'MaxObjectiveEvaluations', 1, 'NumSeedPoints', N0, ...
        %         'PlotFcn', 'all', 'InitialX', InitData, 'AcquisitionFunctionName', 'lower-confidence-bound', 'OutputFcn', @saveToFile, 'SaveFileName', append('/home/mahdi/PhD application/ETH/Rupenyan/code/data_driven_controller/tmp/', FileName));
    end

    InitData_all=[InitData_all; InitData(N0+1:end,:)];
    InitobjectiveData_all = [InitobjectiveData_all; InitobjectiveData(N0+1:end,:)];
    objectiveEstData_all = [objectiveEstData_all; objectiveEstData(N0+1:end,:)];
    objectiveData_all = [objectiveData_all; objectiveEstData(N0+1:end,:)];
    XobjectiveEstData_all = [XobjectiveEstData_all; XobjectiveEstData(N0+1:end,:)];
    XobjectiveData_all = [XobjectiveData_all; XobjectiveEstData(N0+1:end,:)];

    data=data_start;
    InitData=InitData_start;
    InitobjectiveData=InitobjectiveData_start;
    objectiveEstData=objectiveEstData_start;
    objectiveData=objectiveData_start;
    XobjectiveEstData=XobjectiveEstData_start;
    XobjectiveData=XobjectiveData_start;

end
save(append(dir,'InitData_all'),'InitData_all')
save(append(dir,'InitobjectiveData_all.mat'),'InitobjectiveData_all')
save(append(dir,'objectiveEstData_all.mat'),'objectiveEstData_all')
save(append(dir,'objectiveData_all.mat'),'objectiveData_all')
save(append(dir,'XobjectiveEstData_all.mat'),'XobjectiveEstData_all')
save(append(dir,'XobjectiveData_all.mat'),'XobjectiveData_all')

end

function [objective] = myObjfun_ApproxLoop_perturbed(vars, G, G2, Tf, sampleTf, sampleTs, np2, N_real_repeat)
global N
global idx
global data
if isempty(N)
    N=1;
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif N==1
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif N==2
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif N==3
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);
    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif N==4
    N=N+1;
    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;
elseif idx==N_real_repeat
    N = N+1;

    G2 = tfest(data,np2);

    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;     w2=500;     objective=ov/w1+st/w2;

    idx= idx +1;
elseif idx==N_real_repeat+1
    N = N+1;

    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 0.95e0, 0].*(rand(1,1)-0.5));

    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= idx +1;

elseif idx==N_real_repeat+2
    N = N+1;

    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.0e0, 0].*(rand(1,1)-0.5));

    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= idx +1;

elseif idx==N_real_repeat+3
    N = N+1;

    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.05e0, 0].*(rand(1,1)-0.5));

    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= idx +1;

elseif idx==N_real_repeat+4
    N = N+1;
    %     G2tmp = n4sid(data,np2);
    %     [A,B,C,D,~] = idssdata(G2tmp);
    %     [num_surrogate, den_surrogate] = ss2tf(A,B,C,D);
    %     G2 = tf(num_surrogate, den_surrogate);

    G2=tf(G2.Numerator, G2.Denominator+G2.Denominator.*[0, 1.1e0, 0].*(rand(1,1)-0.5));

    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G2, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    idx= 0;

else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    C=tf([vars.Kd+Tf*vars.Kp,vars.Kp+Tf*vars.Ki,vars.Ki], [Tf, 1, 0]);
    CL=feedback(C*G, 1);

    ov=abs(stepinfo(CL).Overshoot);
    st=stepinfo(CL).SettlingTime;
    if isnan(ov) || isinf(ov) || ov>1e3
        ov=1e5;
    end
    if isnan(st) || isinf(st) || st>1e5
        st=1e5;
    end
    w1=1;
    w2=500;
    objective=ov/w1+st/w2;

    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
end


function [objective] = myObjfun_ApproxLoop(vars, G, G2, sampleTf, sampleTs, np2, N_real_repeat)
global N
global idx
global data
if isempty(N)
    N=1;
    objective=myObjfun_Loop(vars, G2);
    idx= 0;
elseif idx==N_real_repeat
    N = N+1;
    G2 = tfest(data,np2);
    objective=myObjfun_Loop(vars, G2);
    idx= 0;
else
    N = N+1;
    %     todo move some lines outside with handler@: faster?
    objective=myObjfun_Loop(vars, G);
    C=tf([vars.Kp,vars.Kp*vars.Ki], [1, 0]);
    CL=feedback(C*G, 1);
    CLU=feedback(C, G);
    ytmp=step(CL,0:sampleTs:sampleTf);
    utmp=step(CLU,0:sampleTs:sampleTf);
    data = merge(data, iddata(ytmp,utmp,sampleTs));
    idx= idx +1;
end
end

function [objective] = myObjfun_Loop(vars, G)
%     todo move some lines outside with handler@: faster?
C=tf([vars.Kp,vars.Kp*vars.Ki], [1, 0]);
CL=feedback(C*G, 1);

ov=abs(stepinfo(CL).Overshoot);
st=stepinfo(CL).SettlingTime;

[y,t]=step(CL);
reference=1;
e=abs(y-reference);
Tr=stepinfo(CL, 'RiseTimeLimits',[0.1,1.0]).RiseTime;
ITAE = trapz(t, t.*abs(e));

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

w1=0.1;
w2=1;
w3=1;
w4=0.5;
objective=ov/w1+st/w2+Tr/w3+ITAE/w4;
end