close all;
clear all;
clc;

addpath(genpath("."));

% loads robot from the urdf file
robot = importrobot('RPR_yyz_renamed.urdf');
% robot configurations are now row vectors
robot.DataFormat = 'col'; 
showdetails(robot);

% symbols init for joint variables
syms q1 q2 q3 real;
syms dq1 dq2 dq3 real;
syms ddq1 ddq2 ddq3 real;

q = [q1;q2;q3];
dq = [dq1;dq2;dq3];
ddq = [ddq1;ddq2;ddq3];

% base to first joint, first to second joint and so on
delta_b_0 = 0.15;
delta_0_1 = 0.4;
delta_1_2 = 0.3;
delta_2_3 = 0.16;

DH_table = [
%  a           alpha   d               theta
   0           pi/2    delta_b_0       0
   delta_0_1   0       0               q1
   0           -pi/2   delta_1_2 + q2  0
   delta_2_3  0       0               q3
];

% to adjust the EE to match the robotics toolbox one we must rotate about
% the y axis by pi/2 rad
rotateEE = [ eul2rotm([0 pi/2 0],"ZYZ") [0; 0; 0]; [0 0 0 1] ];

% converts DH rows to homogeneous transformations
T_dh = DHToTransforms(DH_table);

transforms = [T_dh, rotateEE];
linksName = {    'Base Link',   'Link1',      'Link2',      'Link3',     'EE'       };
jointsType = {     'Fixed',    'Revolute',   'Prismatic',  'Revolute',  'Fixed'    };
framesName = {'b',          '0',         '1',           '2',         '3',      'ee'};

% rinominare T_b_i to T_i_b
T_b_i = cumulateTransforms(T_dh);

CoM = {
    0 applyTransform(T_b_i{2}, [-delta_0_1/2;0;0]) applyTransform(T_b_i{3}, [0;delta_1_2/2;0]) applyTransform(T_b_i{4}, [-delta_2_3/2;0;0]) 0
};

m = [0; 1; 1; 1;0];

I_Link1 = parallelAxis(inertiaVec2tensor(cylinderInertia(m(1),0.02, 0, delta_0_1)), m(1), [-delta_0_1/2;0;0]);
I_Link2 = parallelAxis(inertiaVec2tensor(prismInertia(m(2),0.3, 0.03, delta_1_2)), m(2), [0;0;-delta_1_2/2]);
I_Link3 = parallelAxis(inertiaVec2tensor(cylinderInertia(m(3),0.02, 0, delta_2_3)), m(3), [-delta_2_3/2;0;0]);

MoI = {zeros(3), I_Link1, I_Link2, I_Link3, zeros(3)};

myRobot = loadRobot(transforms, framesName, jointsType, linksName, m, MoI, CoM, [q dq ddq]);

staticConfiguration = [
    -pi/5, 0, 0;...
    -0.18, 0, 0;...
    pi/3, 0, 0;...
];

[aa, torques] = evalRobot(myRobot, staticConfiguration);

