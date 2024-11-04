close all;
% clear all;
clc;

addpath(genpath("."));

% loads robot from the urdf file
robot = importrobot('RPR_yyz.urdf');
robot.DataFormat = 'col'; 
showdetails(robot)

% symbols init
syms q1 dq1 ddq1 q2 dq2 ddq2 q3 dq3 ddq3 real;

% joint variables 




q = [q1 q2 q3]';
dq = [dq1 dq2 dq3]';
ddq = [ddq1 ddq2 ddq3]';

qTypes = {'Revolute', 'Prismatic', 'Revolute'};

% robot machanical properties

% base to first joint, first to second joint and so on
delta_b_0 = 0.15;
delta_0_1 = 0.4;
delta_1_2 = 0.3;
delta_2_ee = 0.16;

DH_table = [
%  a           alpha   d               theta
   0           pi/2    delta_b_0       0
   delta_0_1   0       0               q1
   0           -pi/2   delta_1_2 + q2  0
   delta_2_ee  0       0               q3
];

% to adjust the EE to match the robotics toolbox one we must rotate about
% the y axis by pi/2 rad
rotateEE = [ eul2rotm([0 pi/2 0],"ZYZ") [0; 0; 0]; [0 0 0 1]];

kinematics = loadKinematics(DH_table, rotateEE, q, qTypes);

test_joint_values = [-pi/3 -0.11 pi/8]';

evaluatedKinematics = evaluateKinematics(kinematics, q, test_joint_values);

plot = 0;
if plot
    % plotting the robot from the robotics toolbox and the transforms obtained
    % via the kinematics evaluation
    
    figure(1);
    subplot(131);
    show(robot, test_joint_values');
    subplot(133);
    hold on;
    framesToPlot = [eye(4), evaluatedKinematics.T_cumulative];
    for i=1:size(framesToPlot,2)
       plotTransforms(framesToPlot{i}(1:3,4)',rotm2quat(framesToPlot{i}(1:3, 1:3)), FrameSize=0.1);
    
       if i > 1
            line([framesToPlot{i-1}(1,4);framesToPlot{i}(1,4)],[framesToPlot{i-1}(2,4);framesToPlot{i}(2,4)],[framesToPlot{i-1}(3,4);framesToPlot{i}(3,4)]);
       end
    end
end

% pose from the direct kinematics
testPose = evaluatedKinematics.T_cumulative{end}(1:3, 4);

% manual inverse kinematics
manual_ik_q = invKinematics(testPose);

% robotics toolbox inverse kinematics
ik = inverseKinematics("RigidBodyTree",robot);

ik_q_values = ik('ee',evaluatedKinematics.T_cumulative{end}, [0.1 0.1 0.1 0.1 0.1 0.1], robot.homeConfiguration);

plot = 1;
if plot
    % plotting the robot from the robotics toolbox and the transforms obtained
    % via the kinematics evaluation
    
    figure();
    subplot(131);
    title('Robotics toolbox IK');
    show(robot, ik_q_values);
    subplot(133);
    title('Manual IK');
    show(robot, manual_ik_q);

end


kineticEnergy = 0.5*'