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
rotateEE = [  eul2rotm([0 pi/2 0],"ZYZ") [0; 0; 0]; [0 0 0 1] ];

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

I_Link1_CoM = inertiaVec2tensor(cylinderInertia(m(2),0.02, 0, delta_0_1));
I_Link2_CoM = inertiaVec2tensor(prismInertia(m(3),0.3, 0.03, delta_1_2));
I_Link3_CoM = inertiaVec2tensor(cylinderInertia(m(4),0.02, 0, delta_2_3));

MoI_CoM = {zeros(3), I_Link1_CoM, I_Link2_CoM, I_Link3_CoM, zeros(3)};

I_Link1 = parallelAxis(I_Link1_CoM, m(2), [-delta_0_1/2;0;0]);
I_Link2 = parallelAxis(I_Link2_CoM, m(3), [0;0;-delta_1_2/2]);
I_Link3 = parallelAxis(I_Link3_CoM, m(4), [-delta_2_3/2;0;0]);

MoI = {zeros(3), I_Link1, I_Link2, I_Link3, zeros(3)};

g =  [0;0;-9.81];

% Custom robotic structure
myRobot = loadRobot(transforms, framesName, jointsType, linksName, m, MoI, CoM, [q dq ddq], g);
myRobot.func = getRobotFunctions(myRobot);

myRobot_I_CoM = loadRobot(transforms, framesName, jointsType, linksName, m, MoI_CoM, CoM, [q dq ddq], g);

% Inverse kinematics(closed form)
q1 = @(pose) atan((pose(3) - 0.15)/pose(1));
q3 = @(pose) acos((pose(3)  - 0.15 - 0.4 * sin(q1(pose)))/ (0.16 * sin(q1(pose))));
q2 = @(pose) 0.16 * sin(q3(pose)) - 0.3 - pose(2);

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
    0,   0       -1.5;  % 3
];

if 0
    figure();
    show(robot, zeroTorqueConfiguration(:, 1));
    title('Zero-Torque configuration');
end

[myEvaluatedRobot1, l_torques1] = evalRobot(myRobot, zeroTorqueConfiguration);
[myEvaluatedRobot2, l_torques2] = evalRobot(myRobot, velConfiguration);
[myEvaluatedRobot3, l_torques3] = evalRobot(myRobot, accelConfiguration);

[myEvaluatedRobot1_A, l_torques1_A] = evalRobot(myRobot, PoseAConfiguration);
[myEvaluatedRobot2_A, l_torques2_A] = evalRobot(myRobot, velAConfiguration);
[myEvaluatedRobot3_A, l_torques3_A] = evalRobot(myRobot, accelAConfiguration);

% Inverse kinematics check

poseZero_inv_kin = [ ...
    q1(myEvaluatedRobot1.T_b_i{end}(1:3, 4)); ...
    q2(myEvaluatedRobot1.T_b_i{end}(1:3, 4)); ...
    q3(myEvaluatedRobot1.T_b_i{end}(1:3, 4)); ...
];

poseA_inv_kin = [ ...
    q1(myEvaluatedRobot1_A.T_b_i{end}(1:3, 4)); ...
    q2(myEvaluatedRobot1_A.T_b_i{end}(1:3, 4)); ...
    q3(myEvaluatedRobot1_A.T_b_i{end}(1:3, 4)); ...
];

%% Plots
% figure();
% subplot(121);
% show(robot, PoseAConfiguration(1:3, 1));
% subplot(122);
% show(robot, double(poseA_inv_kin));
% 
% figure();
% subplot(121);
% show(robot, zeroTorqueConfiguration(1:3, 1));
% subplot(122);
% show(robot, double(poseZero_inv_kin));

%%

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

Im = [Im1; Im2; Im3];

Fs = [Fs1; Fs2; Fs3];
Fs_val = [0;0;0];

Fv = [Fv1; Fv2; Fv3];
Fv_val = [0;0;0];

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

[ne_tau1, ne_B, ne_C_vec, ne_G] = newtonEuler(myRobot, he, [0;0;0], [0;0;0], [0;0;0]);

ne_torques1 = subs(ne_tau1,[q;dq;ddq],zeroTorqueConfiguration(:));
ne_torques2 = subs(ne_tau1,[q;dq;ddq;Fs;Fv],[velConfiguration(:);Fs_val; Fv_val]);
ne_torques3 = subs(ne_tau1,[q;dq;ddq;Fs;Fv],[accelConfiguration(:);Fs_val; Fv_val]);

ne_torques1_A = subs(ne_tau1,[q;dq;ddq],PoseAConfiguration(:));
ne_torques2_A = subs(ne_tau1,[q;dq;ddq;Fs;Fv],[velAConfiguration(:);Fs_val; Fv_val]);
ne_torques3_A = subs(ne_tau1,[q;dq;ddq;Fs;Fv],[accelAConfiguration(:);Fs_val; Fv_val]);


%% Dynamics matrices check

ne_B;
myRobot.dynamics.B;

subs(ne_C_vec, [Fs;Fv], [Fs_val;Fv_val]);
myRobot.dynamics.C * dq;

ne_G;
myRobot.dynamics.G;

%% Dynamic model in the operational space

% Ba = inv(myRobot.Ja * inv(myRobot.dynamics.B) * myRobot.Ja');
% Ca_dx = Ba*Ja*inv(myRobot.dynamics.B)*myRobot.dynamics.C*dq - Ba*diff(myRobot.Ja)*dq;
% ga = Ba*myRobot.Ja*inv(B)*myRobot.dynamics.G;
% ua_e = myRobot.Ta'*he;


%% Save the Robot dynamics model
toolboxRobot = robot;

save("robot_model", "myRobot", "toolboxRobot");

%% Plotting
% figure();
% 
% jointsIndex = find(cell2mat(cellfun(@(x) ismember(x, {'Prismatic','Revolute'}), myRobot.jointsType, 'UniformOutput', 0)));
% 
% for i=1:myRobot.DOF
%     R_b_i = myRobot.T_b_i{jointsIndex(i)}(1:3, 1:3);
% 
%     com = subs(CoM{jointsIndex(i)}, [q], PoseAConfiguration(1:3, 1));
%     eval_ddp_c = subs(R_b_i * ddp_c{i}, [q;dq;ddq], PoseAConfiguration(:));
% 
%     plot3(com(1), com(2), com(3), "*");
% 
%     quiver3(com(1), com(2), com(3),eval_ddp_c(1), eval_ddp_c(2), eval_ddp_c(3), 1);
%     hold on;
% end
% 
% plotframe();
% axis equal;
% axis auto;


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

% 
% figure();
% 
% plotRobotConfiguration = [0;0;0];
% 
% text(0,0,0, strcat('$\Sigma_', myRobot.framesName{1}, '$') ,'fontSize', 20 ,  'Interpreter', 'latex');
% hold on;
% 
% for i=2:size(myRobot.framesName, 2)
% 
%     text(0,0,0, strcat('$\Sigma_', myRobot.framesName{i}, '$') ,'fontSize', 20 ,  'Interpreter', 'latex');
% 
% 
%     rt_b_i = double(subs(myRobot.T_b_i{i-1}, myRobot.jointsSymbol(1:3, 1), plotRobotConfiguration));
%     R_b_i = rt_b_i(1:3, 1:3);
%     t_b_i = rt_b_i(1:3, 4);
% 
%     quiver3(R_b_i(1), R_b_i(2), R_b_i(3), t_b_i(1), t_b_i(2), t_b_i(3), 1);
% 
%     hold on;
%     % quiver3(trans_b_i(1), trans_b_i(2), trans_b_i(3), r_c_vec(1), r_c_vec(2), r_c_vec(3),1);
% end
% 
% axis equal;
% axis auto;
% 
% show(robot, homeConfiguration(robot));
% hold on;
% grid off;
% axis off;
% 
% set(gca,'CameraPosition',[9.9720   10.2682    9.7554]);
% set(gca,'CameraTarget',[0.1822   -0.1650   -0.0073]);
% set(gca,'CameraViewAngle',2.5142);
% 
% text(0,0,-0.1, '$\Sigma_b$' ,'fontSize', 20 ,  'Interpreter', 'latex');
% 
% % getTransform(robot, homeConfiguration(robot),"Link2");
% 
% text(0, 0, 0.30, '$\Sigma_0$' ,'fontSize', 20 ,  'Interpreter', 'latex');
% text(0.4, 0, 0.30, '$\Sigma_1$' ,'fontSize', 20 ,  'Interpreter', 'latex');
% text(0.4,  -0.3000,0.28, '$\Sigma_2$' ,'fontSize', 20 ,  'Interpreter', 'latex')
% text(0.65,  -0.3000,0.28, '$\Sigma_3$' ,'fontSize', 20 ,  'Interpreter', 'latex');
% text(0.65,  -0.3000,0.05, '$\Sigma_{ee}$' ,'fontSize', 20 ,  'Interpreter', 'latex');