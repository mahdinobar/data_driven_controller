%% Connect to OPCUA
clear all; clc;
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

actPos_all=[];
actVel_all=[];
actCur_all=[];
r_all=[];
t_all=[];
for exe=1:200
    disp(exe)

    % Set P and D gain
    write_OPCUA(uaObj,'LQR_P_x', 7000);
    write_OPCUA(uaObj,'LQR_D_v', 76.6);
    pause(0.2);

    % perform experiment
    write_OPCUA(uaObj,'Go', 1); % Perform Experiment
    pause(7.5);

    % read the results
    actPos = read_OPCUA(uaObj,'arrActPos')';
    actVel = read_OPCUA(uaObj,'arrActVel')';
    actCur = read_OPCUA(uaObj,'arrActCur')';

    %     figure
    %     hold on
    %     plot(t,r,'k');
    %     plot(t,actPos,'b');
    %     xlabel('Time [s]'); ylabel('Pos [m]')
    %
    %     figure
    %     plot(actVel)
    %     xlabel('Time [s]'); ylabel('Vel [m/s]')

    actPos_all=[actPos_all,actPos];
    actVel_all=[actVel_all,actVel];
    actCur_all=[actCur_all,actCur];
    r_all=[r_all,r'];
    t_all=[t_all,t'];


    % figure
    % hold on
    % plot(actCur)
    % plot(actNomCur)
    % xlabel('Time [s]'); ylabel('Current [A]')
    % legend('actCur','NomCur')

    % time = linspace(0,length(actPos)/1000,length(actPos))';

    % ExprData = timetable(actNomCur,actCur,actVel,actPos,'SampleRate',1000);
    % save('ExprData\Expr_9.mat','ExprData')

    exp_data.actPos_all=actPos_all;
    exp_data.actVel_all=actVel_all;
    exp_data.actCur_all=actCur_all;
    exp_data.r_all=r_all;
    exp_data.t_all=t_all;
    save('C:\Users\nobar\data_driven_controller\linear_motor\exp_data.mat','exp_data')
end
%%
% Disconnect from OPCUA
disconnect_OPCUA(uaObj)