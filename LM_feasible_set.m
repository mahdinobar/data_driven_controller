%% Connect to OPCUA
clear all; clc;close all
%%
addpath('C:\Users\nobar\data_driven_controller\linear_motor')
ip='192.168.188.21';
sInfo = opcuaserverinfo(ip); % SPS Name
% sInfo = opcuaserverinfo('192.168.188.21'); % SPS Name
% Create Client
uaObj = opcua(sInfo.Hostname,sInfo.Port);
% Connect to OPC server
connect(uaObj,'tm','sX7Cswc34wjvwq'); % SPS Loggin
%% Load Trajectory
% load("dem_x/dem_x_9_mm.mat")
t = (0.001:0.001:7);
r= 10.*(t>2)+30-10.*(t>5);

% % set reference input
write_OPCUA(uaObj,'arrDemPos', r);
write_OPCUA(uaObj,'arrShowPos', r);
pause(1);
%%
s = tf('s');
Kp = 0.44262;
Tp = 0.070983;
Td = 0.001;
Gp_x = Kp/(1 * Tp *s) * 1/s * exp(-Td*s);
F=0.001;
%%
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_feasible.mat','exp_data')
actPos_all=[];
actVel_all=[];
actCur_all=[];
r_all=[];
t_all=[];
for D=10:10:90
    for P=500:500:12000
        C=P+D*s/(F*s+1);
        CLsys = feedback(Gp_x*C,1);
        [Gm,Pm,Wcg,Wcp] = margin(CLsys);
        if Pm>20
            % Set P and D gain
%             write_OPCUA(uaObj,'LQR_P_x', P);
%             write_OPCUA(uaObj,'LQR_D_v', D);
%             pause(0.2);
% 
%             % perform experiment
%             write_OPCUA(uaObj,'Go', 1); % Perform Experiment
%             pause(7.5);
% 
%             % read the results
%             actPos = read_OPCUA(uaObj,'arrActPos')';
%             actVel = read_OPCUA(uaObj,'arrActVel')';
%             actCur = read_OPCUA(uaObj,'arrActCur')';
            t = (0.001:0.001:7);
            r= 10.*(t>2)+30-10.*(t>5);
        else
            disp(Pm)
%             actPos=-1.*ones(7000,1);
%             actVel=-1.*ones(7000,1);
%             actCur=-1.*ones(7000,1);
            r=-1.*ones(1,7000);
            t=-1.*ones(1,7000);
        end

%         actPos_all=[actPos_all,actPos];
%         actVel_all=[actVel_all,actVel];
%         actCur_all=[actCur_all,actCur];
        r_all=[r_all,r'];
        t_all=[t_all,t'];
%         exp_data.actPos_all=actPos_all;
%         exp_data.actVel_all=actVel_all;
%         exp_data.actCur_all=actCur_all;
        exp_data.r_all=r_all;
        exp_data.t_all=t_all;
        save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_33.mat','exp_data')
    end
end
%%
% Disconnect from OPCUA
disconnect_OPCUA(uaObj)

%%
% %              Kp
% %   G(s) = ---------- * exp(-Td*s)
% %           1+Tp1*s
% %         Kp = 0.44262
% %        Tp1 = 0.070983
% %         Td = 0.001334        s = tf('s');
% Kp = 0.44262;
% Tp = 0.070983;
% Td = 0.001;
% Gp_x = Kp/(1 * Tp *s) * 1/s * exp(-Td*s);
% counter=0;
% failed=0;
% for D=10:1:90
%     for P=500:100:15000
%         counter=counter+1
%         F=0.0002;
%         C=P+D*s/(F*s+1);
%         CLsys = feedback(Gp_x*C,1);
%         [Gm,Pm,Wcg,Wcp] = margin(CLsys);
%         if Pm<20
%             disp(Pm)
%             failed=failed+1;
%         end
%     end
% end


%%
t = (0.001:0.001:7);
r= 10.*(t>2)+30-10.*(t>5);

% % set reference input
write_OPCUA(uaObj,'arrDemPos', r);
write_OPCUA(uaObj,'arrShowPos', r);
pause(1);
%%

P_all=[];
D_all=[];
for D=10:10:90
    for P=500:500:12000
        P_all=[P_all,P];
        D_all=[D_all,D];
    end
end
load('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_feasible.mat','exp_data')
exp_data.P=P_all;
exp_data.D=D_all;
save('/home/mahdi/ETHZ/GBO/code/data_driven_controller/linear_motor/exp_data_feasible_3.mat','exp_data')


