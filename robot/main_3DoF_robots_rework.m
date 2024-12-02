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

I_Link1_CoM = inertiaVec2tensor(cylinderInertia(m(1),0.02, 0, delta_0_1));
I_Link2_CoM = inertiaVec2tensor(prismInertia(m(2),0.3, 0.03, delta_1_2));
I_Link3_CoM = inertiaVec2tensor(cylinderInertia(m(3),0.02, 0, delta_2_3));

MoI_CoM = {zeros(3), I_Link1_CoM, I_Link2_CoM, I_Link3_CoM, zeros(3)};

I_Link1 = parallelAxis(I_Link1_CoM, m(1), [-delta_0_1/2;0;0]);
I_Link2 = parallelAxis(I_Link2_CoM, m(2), [0;0;-delta_1_2/2]);
I_Link3 = parallelAxis(I_Link3_CoM, m(3), [-delta_2_3/2;0;0]);

MoI = {zeros(3), I_Link1, I_Link2, I_Link3, zeros(3)};

g =  [0;0;-9.81];

% Custom robotic structure
myRobot = loadRobot(transforms, framesName, jointsType, linksName, m, MoI, CoM, [q dq ddq], g);

myRobot_I_CoM = loadRobot(transforms, framesName, jointsType, linksName, m, MoI_CoM, CoM, [q dq ddq], g);

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
    pi/2,  0,    0;   % 1
    0,     0,    0;   % 2
    0,     0,    0;   % 3
];

PoseAConfiguration = [
%   q      dq    ddq
    pi/4,  0,    0;   % 1
    -0.17,     0,    0;   % 2
    -pi/9,     0,    0;   % 3
];

velConfiguration = zeroTorqueConfiguration + [
%   q    dq      ddq
    0,   -10.5,   0;... % 1
    0,   10.5,    0;... % 2
    0,   5.5,    0;...  % 3
];

accelConfiguration = zeroTorqueConfiguration + [
%   q    dq      ddq
    0,   0       10.5;  % 1
    0,   0       -10.5; % 2
    0,   0       -1.5; % 3
];

velAConfiguration = PoseAConfiguration + [
%   q    dq      ddq
    0,   -10.5,   0;... % 1
    0,   10.5,    0;... % 2
    0,   5.5,    0;...  % 3
];

accelAConfiguration = PoseAConfiguration + [
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

[myEvaluatedRobot1_A, l_torques1_A] = evalRobot(myRobot, PoseAConfiguration);
[myEvaluatedRobot2_A, l_torques2_A] = evalRobot(myRobot, velAConfiguration);
[myEvaluatedRobot3_A, l_torques3_A] = evalRobot(myRobot, accelAConfiguration);

% Robotics toolbox inverse dynamics
toolbox_toques1 = inverseDynamics(robot, zeroTorqueConfiguration(:, 1),zeroTorqueConfiguration(:, 2),zeroTorqueConfiguration(:, 3));
toolbox_toques2 = inverseDynamics(robot, velConfiguration(:, 1),velConfiguration(:, 2),velConfiguration(:, 3));
toolbox_toques3 = inverseDynamics(robot, accelConfiguration(:, 1),accelConfiguration(:, 2),accelConfiguration(:, 3));

toolbox_toques1_A = inverseDynamics(robot, PoseAConfiguration(:, 1),PoseAConfiguration(:, 2),PoseAConfiguration(:, 3));
toolbox_toques2_A = inverseDynamics(robot, velAConfiguration(:, 1),velAConfiguration(:, 2),velAConfiguration(:, 3));
toolbox_toques3_A = inverseDynamics(robot, accelAConfiguration(:, 1),accelAConfiguration(:, 2),accelAConfiguration(:, 3));

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
myRobot_I_CoM.motors = {motor1, motor2, motor3};
myRobot.frictions = {friction1, friction2, friction3};
myRobot_I_CoM.frictions = {friction1, friction2, friction3};

[ne_tau1, r, r_c, ddp_c] = newtonEuler(myRobot_I_CoM, he, [0;0;0], [0;0;0], [0;0;0]);

ne_torques1 = subs(ne_tau1,[q;dq;ddq],zeroTorqueConfiguration(:));
ne_torques2 = subs(ne_tau1,[q;dq;ddq],velConfiguration(:));
ne_torques3 = subs(ne_tau1,[q;dq;ddq],accelConfiguration(:));

ne_torques1_A = subs(ne_tau1,[q;dq;ddq],PoseAConfiguration(:));
ne_torques2_A = subs(ne_tau1,[q;dq;ddq],velAConfiguration(:));
ne_torques3_A = subs(ne_tau1,[q;dq;ddq],accelAConfiguration(:));


figure();

jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), myRobot.jointsType, 'UniformOutput', 0)));

for i=1:myRobot.DOF
    R_b_i = myRobot.T_b_i{jointsIndex(i)}(1:3, 1:3);

    com = subs(CoM{jointsIndex(i)}, [q], PoseAConfiguration(1:3, 1));
    eval_ddp_c = subs(R_b_i * ddp_c{i}, [q;dq;ddq], PoseAConfiguration(:));

    plot3(com(1), com(2), com(3), "*");

    quiver3(com(1), com(2), com(3),eval_ddp_c(1), eval_ddp_c(2), eval_ddp_c(3), 1);
    hold on;
end

plotframe();
axis equal;
axis auto;


% figure();
% 
% jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), myRobot.jointsType, 'UniformOutput', 0)));
% 
% for i=1:myRobot.DOF
%     trans_b_i = double(subs(myRobot.T_b_i{jointsIndex(i)}(1:3, 4), q, zeroTorqueConfiguration(1:3, 1)));
%     R_b_i = myRobot.T_b_i{jointsIndex(i)}(1:3, 1:3);
% 
%     r_vec = double(subs(R_b_i*r{i}, q, zeroTorqueConfiguration(1:3, 1)));
%     r_c_vec = double(subs(R_b_i*r_c{i}, q, zeroTorqueConfiguration(1:3, 1)));
% 
%     quiver3(trans_b_i(1), trans_b_i(2), trans_b_i(3), r_vec(1), r_vec(2), r_vec(3), 1);
%     hold on;
%     quiver3(trans_b_i(1), trans_b_i(2), trans_b_i(3), r_c_vec(1), r_c_vec(2), r_c_vec(3),1);
% end
% 
% axis equal;
% axis auto;

