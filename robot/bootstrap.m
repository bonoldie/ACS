% Bootstrap the simulink environment
clear all;

addpath(genpath('.'));
load('robot_model.mat');
robot_tb = importrobot('RPR_yyz_renamed.urdf');
robot_tb.DataFormat = 'column';

q_i = [pi/2;0;pi/3];
dq_i = [0;0;0];


Kp = eye(myRobot.DOF)*50;
Kd = eye(myRobot.DOF)*20;
Ki = eye(myRobot.DOF)*5;

Kp_op = diag([50 50 50 0.2 0.2 0.2]);
Kd_op = diag([20 20 20 0.1 0.1 0.1]);
Ki_op = diag([10 10 10 0 0 0]);

