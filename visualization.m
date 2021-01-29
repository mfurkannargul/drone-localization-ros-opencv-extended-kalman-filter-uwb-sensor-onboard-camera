clear all, close all, clc

bag = rosbag('rostopic_output.bag');
rosbag info 'rostopic_output.bag'

% % EKF
b_EKF = select(bag,'Topic','/ekf');
msgStructsEKF = readMessages(b_EKF,'DataFormat','struct');
msgStructsEKF{1}
x_EKF = cellfun(@(m) double(m.Linear.X),msgStructsEKF);
y_EKF = cellfun(@(m) double(m.Linear.Y),msgStructsEKF);

% Mavros
b_Mavros = select(bag,'Topic','/mavros/vision_pose/pose');
msgStructsMavros = readMessages(b_Mavros,'DataFormat','struct');
msgStructsMavros{1} 
x_Mavros = cellfun(@(m) double(m.Pose.Position.X),msgStructsMavros);
y_Mavros = cellfun(@(m) double(m.Pose.Position.Y),msgStructsMavros);

% % QR det
b_QR_det = select(bag,'Topic','/qr_det');
msgStructsMQR_det = readMessages(b_QR_det,'DataFormat','struct');
msgStructsMQR_det{1}
x_QR_det = cellfun(@(m) double(m.Linear.X),msgStructsMQR_det);
y_QR_det = cellfun(@(m) double(m.Linear.Y),msgStructsMQR_det);


% % % QR raw
bQR_raw = select(bag,'Topic','/qr_raw');
msgStructsMQR_raw = readMessages(bQR_raw,'DataFormat','struct');
msgStructsMQR_raw{1}
x_QR_raw = cellfun(@(m) double(m.Linear.X),msgStructsMQR_raw);
y_QR_raw = cellfun(@(m) double(m.Linear.Y),msgStructsMQR_raw);

% UWB
buwb = select(bag,'Topic','/uwb');
msgStructs_uwb = readMessages(buwb,'DataFormat','struct');
msgStructs_uwb{1}
d_uwb = cellfun(@(m) double(m.Data),msgStructs_uwb);

% Vicon
b_vicon = select(bag,'Topic','/p_vicon');
msgStructsp_vicon = readMessages(b_vicon,'DataFormat','struct');
msgStructsp_vicon{1}
x_vicon = cellfun(@(m) double(m.Linear.X),msgStructsp_vicon);
y_vicon = cellfun(@(m) double(m.Linear.Y),msgStructsp_vicon);

% P_mav
b_P_mav = select(bag,'Topic','/p_mav');
msgStructsp_P_mav = readMessages(b_P_mav,'DataFormat','struct');
msgStructsp_P_mav{1}
x_P_mav = cellfun(@(m) double(m.Linear.X),msgStructsp_P_mav);
y_P_mav = cellfun(@(m) double(m.Linear.Y),msgStructsp_P_mav);

b_meas_mode  = select(bag,'Topic','/meas_mode');
msgStructsp_meas_mode  = readMessages(b_meas_mode,'DataFormat','struct');
msgStructsp_meas_mode{1}
if_measured = cellfun(@(m) double(m.Linear.X),msgStructsp_meas_mode);

%%
figure(5)
% axis([-1.5 2.5 -2.2 2.2])
xlabel("X Position")
ylabel("Y Position")
title("Real Time Drone Localization Results")
hold on
start =150
start = 1
figure(5)
axis([-2.6 2 -3.68 2.42])
hold on
% 6.1
% 4.6 
x0=0;
y0=0;
width=0.25;
height=0.25;
for t=start:7:1000
    h1 = plot(-x_EKF(start:2:t,1),-y_EKF(start:2:t,1), 'rx', 'Linewidth',1.5)
    hold on
    h2 = plot(-x_P_mav(start:2:t,1),-y_P_mav(start:2:t,1),'g--','Linewidth',2)
    if if_measured(t) == 0
        hold on
        h3 = plot(-x_QR_det(t,1),-y_QR_det(t,1),'k+','Linewidth',1.8, 'Markersize', 10)
    end
    legend([h1 h2 h3], 'EKF', 'Ground Truth', 'Camera-Based','position',[x0 y0 width height]);
    pause(0.1)
end

