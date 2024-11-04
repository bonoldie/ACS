close all;
clear all;
clc;

addpath(genpath("."));

% loads robot from the urdf file
robot = importrobot('RPR_yyz.urdf');
robot.DataFormat = 'col'; 
showdetails(robot);

% symbols init
syms q1 q2 q3 real;

% joint variables 




q = [q1 q2 q3]';

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
qTypes = {'Revolute', 'Prismatic', 'Revolute'};
frameNames = {'b', '0', '1', '2', '3', 'ee'};

kinematics = loadKinematics(transforms, frameNames, q, qTypes);
% rinominare T_b_i to T_i_b
T_b_i = cumulateTransforms(kinematics.T);

p_CoM = {
    applyTransform(T_b_i{2},[-delta_0_1/2;0;0]);
    applyTransform(T_b_i{3},[0;delta_1_2/2;0]);
    applyTransform(T_b_i{4},[-delta_2_3/2;0;0]);
};

test_q_values = [-pi/5 -0.18 pi/3]';
evaluatedKinematics = evaluateKinematics(kinematics, test_q_values);

plot = 1;
if plot
    % plotting the robot from the robotics toolbox and the transforms obtained
    % via the kinematics evaluation
    
    figure(1);
    subplot(131);
    show(robot, test_q_values);
    subplot(133);
    hold on;

    framesToPlot=cell(1, size(frameNames,2));

    for i=1:size(frameNames,2)
        framesToPlot{i} = transformFromTo(evaluatedKinematics,'b', frameNames{i});
    end


    for i=1:size(frameNames,2)
       plotTransforms(framesToPlot{i}(1:3,4)',rotm2quat(framesToPlot{i}(1:3, 1:3)), FrameSize=0.1);
    
       if i > 1
            line([framesToPlot{i-1}(1,4);framesToPlot{i}(1,4)],[framesToPlot{i-1}(2,4);framesToPlot{i}(2,4)],[framesToPlot{i-1}(3,4);framesToPlot{i}(3,4)]);
       end
    end

    axis equal;
    grid on;

    for i=1:size(p_CoM)
        p_CoM_evaluated = double(subs(p_CoM{i}, q, test_q_values));
        quiver3(0,0,0,p_CoM_evaluated(1),p_CoM_evaluated(2),p_CoM_evaluated(3), 0);
        plot3(p_CoM_evaluated(1),p_CoM_evaluated(2),p_CoM_evaluated(3), '*');
    end
end

%% Direct and inverse kinematics

Tcumulate = cumulateTransforms(evaluatedKinematics.T);

% pose from the direct kinematics
testPose = Tcumulate{end}(1:3, 4);

% manual inverse kinematics
manual_ik_q = invKinematics(testPose);

% robotics toolbox inverse kinematics
ik = inverseKinematics("RigidBodyTree",robot);

ik_q_values = ik('ee',Tcumulate{end}, [0.1 0.1 0.1 0.1 0.1 0.1], robot.homeConfiguration);

plot = 0;
if plot
    % plotting the robot from the robotics toolbox and the transforms obtained
    % via the kinematics evaluation
    
    figure();
    subplot(131);
    show(robot, ik_q_values);
    title('Robotics toolbox IK');
    subplot(133);
    show(robot, manual_ik_q);
    title('Manual IK');
end

%% Partial geometric jacobian  tests 

partialJg_1 = partialJg(kinematics, p_CoM, 1);

partialJg_2 = partialJg(kinematics, p_CoM, 2);

partialJg_3 = partialJg(kinematics, p_CoM, 3);

Jg_b_1 = partialJg(kinematics, {Tcumulate{1}(1:3,4);Tcumulate{2}(1:3,4);Tcumulate{3}(1:3,4)}, 1);
Jg_b_1 = subs(Jg_b_1, q, test_q_values);

Jg_b_1_tb = geometricJacobian(robot, test_q_values, 'Link2');

Jg_b_2 = partialJg(kinematics, {Tcumulate{1}(1:3,4);Tcumulate{2}(1:3,4);Tcumulate{3}(1:3,4)}, 2);
Jg_b_2 = subs(Jg_b_2, q, test_q_values);

Jg_b_2_tb = geometricJacobian(robot, test_q_values, 'Link3');

Jg_b_3 = partialJg(kinematics, {Tcumulate{1}(1:3,4);Tcumulate{2}(1:3,4);Tcumulate{3}(1:3,4)}, 3);
Jg_b_3 = subs(Jg_b_3, q, test_q_values);

Jg_b_3_tb = geometricJacobian(robot, test_q_values, 'Link4');

% Jg_b_2 = geometricJacobianTo(evaluatedKinematics, '2');
% geometricJacobian(robot, test_q_values, 'Link3');
% 
% Jg_b_3 = geometricJacobianTo(evaluatedKinematics, '3');
% geometricJacobian(robot, test_q_values, 'Link4');



%% Dynamics

Tcumulate = cumulateTransforms(kinematics.T);

test_dq_values = [0.5;0.5;0.5];

m = [1;1;1];

I_1 = parallelAxis(inertiaVec2tensor(cylinderInertia(m(1),0.02, 0, 0.4)), m(1), [-0.2;0;0]);
I_2 = parallelAxis(inertiaVec2tensor(prismInertia(m(2),0.3, 0.03, 0.03)), m(2), [0;0;-0.15]);
I_3 = parallelAxis(inertiaVec2tensor(cylinderInertia(m(3),0.02, 0, 0.16)), m(3), [-0.08;0;0]);

Y_pi_mezzi = eul2rotm([0 pi/2 0], 'ZYZ');

I_2 = Y_pi_mezzi * I_2  * Y_pi_mezzi';

inertiaTensors = { 
    I_1,
    I_2,
    I_3
};

robot.Bodies{1}.CenterOfMass = [delta_0_1/2;0;0];
robot.Bodies{1}.Mass = m(1);
robot.Bodies{1}.Inertia = inertiaTensor2vec(inertiaTensors{1});

robot.Bodies{2}.CenterOfMass = [0;0;delta_1_2/2];
robot.Bodies{2}.Mass = m(2);
robot.Bodies{2}.Inertia = inertiaTensor2vec(inertiaTensors{2});

robot.Bodies{3}.CenterOfMass = [delta_2_3/2;0;0];
robot.Bodies{3}.Mass = m(3);
robot.Bodies{3}.Inertia = inertiaTensor2vec(inertiaTensors{3});

robot.Gravity = [0;0;-9.81];

B = sym(zeros(3));

for i=1:kinematics.DOF
    partialGeometricJ = partialJg(kinematics, p_CoM, i);
    Ri = Tcumulate{i+1}(1:3,1:3);
    B = B + (...
        m(i) * partialGeometricJ(1:3,:)' * partialGeometricJ(1:3,:) + ... 
        partialGeometricJ(4:6, :)' * Ri * inertiaTensors{i} * Ri' * partialGeometricJ(4:6, :)...
    );
end

B = simplify(B);

solve(simplify(eig(B)) > 0, [q3], 'ReturnConditions', true);

%  B = inertiaMatrix(kinematics, inertiaTensors);

kineticEnergy = 0.5*test_dq_values'*double(subs(B,q, test_q_values))*test_dq_values;

gVec = [0;0;-9.81];

U = sym(0);

for i=1:kinematics.DOF
    U = U - (m(i) * gVec' * p_CoM{i});
end

U = simplify(U);

potentialEnergy = double(subs(U,q,test_q_values));

totalEnergy = kineticEnergy + potentialEnergy;

%% Equations of motions

% syms t dq1(t) ddq1(t) dq2(t) ddq2(t) dq3(t) ddq3(t) real;
% syms tau1(t) tau2(t) tau3(t) real;
syms t real;
syms q1_t(t) q2_t(t) q3_t(t) real;
syms dq1_t(t) dq2_t(t) dq3_t(t) real;
syms ddq1_t(t) ddq2_t(t) ddq3_t(t) real;

syms tau1 tau2 tau3 real;

q_t = [q1_t;q2_t;q3_t];
dq_t = [dq1_t;dq2_t;dq3_t];
ddq_t = [ddq1_t;ddq2_t;ddq3_t];

% dq_t = diff(q_t, t);
% ddq_t = diff(dq_t, t);

B_q = subs(B, q, q_t);
test_ddq_values = [0.5;0.5;0.5];

eq_motion = B_q * ddq_t;
eq_motion = eq_motion + subs(diff(B_q, t), diff(q_t, t), dq_t)*dq_t;
eq_motion = eq_motion - 0.5 * [...
    diff(transpose(diff(q_t, t)) * B_q * diff(q_t, t), q1_t);
    diff(transpose(diff(q_t, t)) * B_q * diff(q_t, t), q2_t);
    diff(transpose(diff(q_t, t)) * B_q * diff(q_t, t), q3_t);
];
eq_motion = eq_motion + [...
    diff(subs(U, q, q_t), q1_t);
    diff(subs(U, q, q_t), q2_t);
    diff(subs(U, q, q_t), q3_t);
];

eq_motion = subs(eq_motion,diff(q_t, t), dq_t);

tau = subs(eq_motion, [formula(q_t);formula(dq_t);formula(ddq_t)], [test_q_values;test_dq_values; test_ddq_values]);

invDynamics = robot.inverseDynamics(test_q_values, test_dq_values, test_ddq_values);
