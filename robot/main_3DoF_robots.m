close all;
clear all;
clc;

% robot = importrobot('iiwa14.urdf');

robot = importrobot('RPR_yyz ENRICO BONOLDI.urdf');

% robot.DataFormat = 'column';
% config = [0.1, pi/3, -0.15]';

showdetails(robot);

figure(1); 
config = homeConfiguration(robot);

config(1).JointPosition = 0;
config(2).JointPosition = 0;
config(3).JointPosition = 0;

show(robot,config);
title('Robotics toolbox');

xlim([-0.5 0.8]);
ylim([-0.5 0.5]);
zlim([0 0.8]);

eePose = getTransform(robot,config,"ee");

% EE orientation, euler angles in the ZYX axis order
eeOrientation = rotm2eul(eePose(1:3,1:3));

eePosition = eePose(1:3, 4);

disp('EE orientation');
disp(eeOrientation);

disp('EE position');
disp(eePosition);


