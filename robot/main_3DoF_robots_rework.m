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

g =  [0;0;-9.81];

% Custom robotic structure
myRobot = loadRobot(transforms, framesName, jointsType, linksName, m, MoI, CoM, [q dq ddq], g);

% myRobot.motors = {loadMotor}

% Robotic systems toolbox setup
robot.getBody('Link1').Mass = m(2);
robot.getBody('Link2').Mass = m(3);
robot.getBody('Link3').Mass = m(4);

robot.getBody('Link1').Inertia = inertiaTensor2vec(I_Link1);
robot.getBody('Link2').Inertia = inertiaTensor2vec(I_Link2);
robot.getBody('Link3').Inertia = inertiaTensor2vec(I_Link3);

robot.getBody('Link1').CenterOfMass = [delta_0_1/2;0;0];
robot.getBody('Link2').CenterOfMass = [0;0;delta_1_2/2];
robot.getBody('Link3').CenterOfMass = [delta_2_3/2;0;0];

robot.Gravity = g;

%% Models evaluation

zeroTorqueConfiguration = [
%   q      dq    ddq
    pi/2,  0,    0; % 1
    0,     0,    0; % 2
    0,     0,    0; % 3
];

velConfiguration = zeroTorqueConfiguration + [
%   q    dq      ddq
    0,   -10.5,   0;... % 1
    0,   10.5,    0;... % 2
    0,   1.5,    0;... % 3
];

accelConfiguration = zeroTorqueConfiguration + [
%   q    dq      ddq
    0,   0       10.5;  % 1
    0,   0       -10.5; % 2
    0,   0       -1.5; % 3
];

if 0
    figure();
    show(robot, zeroTorqueConfiguration(:, 1));
    title('Zero-Torque configuration');
end

% Inverse dynamics via the Lagrange model
[myEvaluatedRobot1, l_torques1] = evalRobot(myRobot, zeroTorqueConfiguration);
[myEvaluatedRobot2, l_torques2] = evalRobot(myRobot, velConfiguration);
[myEvaluatedRobot3, l_torques3] = evalRobot(myRobot, accelConfiguration);

% Robotics toolbox inverse dynamics
toolbox_toques1 = inverseDynamics(robot, zeroTorqueConfiguration(:, 1),zeroTorqueConfiguration(:, 2),zeroTorqueConfiguration(:, 3));
toolbox_toques2 = inverseDynamics(robot, velConfiguration(:, 1),velConfiguration(:, 2),velConfiguration(:, 3));
toolbox_toques3 = inverseDynamics(robot, accelConfiguration(:, 1),accelConfiguration(:, 2),accelConfiguration(:, 3));

% Newton-Euler recursive method for inverse dynamics

he = [0 0 0 0 0 0]';

syms k1 k2 k3 real
syms Im1 Im2 Im3 real
syms Fv1 Fs1 Fv2 Fs2 Fv3 Fs3 real

motor1 = loadMotor(k1, Im1, [0;0;1]);
friction1 = loadFriction(Fv1, Fs1);
motor2 = loadMotor(k2, Im2, [0;0;1]);
friction2 = loadFriction(Fv2, Fs2);
motor3 = loadMotor(k3, Im3, [0;0;1]);
friction3 = loadFriction(Fv3, Fs3);

myRobot.motors = {motor1, motor2, motor3};
myRobot.frictions = {friction1, friction2, friction3};

[ne_torques1] = newtonEuler(myRobot, he, [0;0;0], [0;0;0], [0;0;0]);


