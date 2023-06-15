%% Connect to OPCUA
clear all; clc; close all
%%
addpath('C:\Users\nobar\data_driven_controller\linear_motor')
addpath('C:\Users\nobar\data_driven_controller\linear_motor\OPCUA')
ip='192.168.188.21';
sInfo = opcuaserverinfo(ip); % SPS Name
% Create Client
uaObj = opcua(sInfo.Hostname,sInfo.Port);
% Connect to OPC server
connect(uaObj,'tm','sX7Cswc34wjvwq'); % SPS Loggin
%% Load Trajectory
% load("dem_x/dem_x_9_mm.mat")
t = (0.001:0.001:7);
r= 10.*(t>2)+30-10.*(t>5);
% set reference input
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
actPos_all=[];
actVel_all=[];
actCur_all=[];
r_all=[];
t_all=[];
PGM_all=[];
%   first round
for D=41:1:54
    for P=5300:60:6120
%   second round
% for D=41.5:1:54.5
%     for P=5330:60:6150
%   third round
% for D=41.25:1:54.25
%     for P=5315:60:6135
%   forth round
% for D=41.75:1:54.75
%     for P=5345:60:6165
            Set P and D gain
            write_OPCUA(uaObj,'LQR_P_x', P);
            write_OPCUA(uaObj,'LQR_D_v', D);
            pause(0.2);

            % perform experiment
            write_OPCUA(uaObj,'Go', 1); % Perform Experiment
            pause(7.5);

            % read the results
            actPos = read_OPCUA(uaObj,'arrActPos')';
            actVel = read_OPCUA(uaObj,'arrActVel')';
            actCur = read_OPCUA(uaObj,'arrActCur')';

        P_all=[P_all,P];
        D_all=[D_all,D];
        actPos_all=[actPos_all,actPos];
        actVel_all=[actVel_all,actVel];
        actCur_all=[actCur_all,actCur];

        exp_data.actPos_all=actPos_all;
        exp_data.actVel_all=actVel_all;
        exp_data.actCur_all=actCur_all;
        exp_data.P_all=P_all;
        exp_data.D_all=D_all;
        save('C:\Users\nobar\data_driven_controller\linear_motor\exp_data_offline_dataset_1.mat','exp_data')
%         % second round
%         save('C:\Users\nobar\data_driven_controller\linear_motor\exp_data_offline_dataset_2.mat','exp_data')
%         % second round
%         save('C:\Users\nobar\data_driven_controller\linear_motor\exp_data_offline_dataset_3.mat','exp_data')
%         % second round
%         save('C:\Users\nobar\data_driven_controller\linear_motor\exp_data_offline_dataset_4.mat','exp_data')
    end
end
%%
% Disconnect from OPCUA
disconnect_OPCUA(uaObj)