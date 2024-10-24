close all;
clear all;
clc;

% imports
run('utils.m');

% loads robot from URDF
robot = importrobot('RPR_yyz ENRICO BONOLDI.urdf');

% robot links length
% base to first joint, first to second joint and so on
delta_b = 0.15;
delta_1 = 0.4;
delta_2 = 0.3;
delta_3 = 0.16;

showdetails(robot);

% Forward kinematics
% defining joint variables as symbols

syms q1 q2 q3 real;

DH_table = [
%  a         alpha   d              theta
   0         pi/2    delta_b        0
   delta_1   0       0              q1
   0         -pi/2   delta_2 + q2   0
   delta_3   0       0              q3
];

% initialize the transformations cell array
% at index i there is the homogeneous transformation from the i to i+1 joint 
DH_transforms = cell(1, size(DH_table,1));

for i=1:size(DH_table,1)
    DH_transforms{i} = subs(homogeneous_transform, [a alpha d theta], DH_table(i, :));
end

% this rotation is needed to align the robot ee with the one we get following the DH convention (rotate about y by an angle of pi/2)
rotateEE = [eul2rotm([0 pi/2 0]) [0;0;0]; 0 0 0 1];

% complete forward kinematics matrix
T_b_ee = eye(4);

for i=1:size(DH_transforms,2)
    T_b_ee = T_b_ee * DH_transforms{i};
end

T_b_ee = T_b_ee * rotateEE;

%% compare robotics toolbox and calculated pose

q1_test_value = pi/4;
q2_test_value = -0.2;
q3_test_value = pi/4;

config = homeConfiguration(robot);

config(1).JointPosition = q1_test_value;
config(2).JointPosition = q2_test_value;
config(3).JointPosition = q3_test_value;

eval_T_b_ee = double(subs(T_b_ee,[q1 q2 q3], [q1_test_value q2_test_value q3_test_value]));

printPose(getTransform(robot,config,"ee"), eval_T_b_ee);

% plot robot model and DH frames
figure; 

subplot(131);
show(robot,config);
title('Robotics toolbox');

xlim([-0.5 0.8]);
ylim([-0.5 0.5]);
zlim([0 0.8]);

subplot(133);
hold on;
title('Forward kinematics');

% base to n_th joint tranform
T_b_n = cell(size(DH_transforms));

for i=1:size(DH_transforms, 2)
    T_b_i = double(subs(DH_transforms{i},[q1 q2 q3], [q1_test_value q2_test_value q3_test_value]));

    if i == 1
        T_b_n{i} = T_b_i;
    else
        T_b_n{i} = T_b_n{i-1} * T_b_i; 
    end
end

% plot axes origin
plotTransforms([0 0 0], [1 0 0 0], FrameSize=0.1);

for i=1:size(T_b_n,2)
   plotTransforms(T_b_n{i}(1:3,4)',rotm2quat(T_b_n{i}(1:3, 1:3)), FrameSize=0.1);

   if i > 1
        % line([T_b_n{i-1}(1,4);T_b_n{i}(1,4)],[T_b_n{i}(2,4);T_b_n{i-1}(2,4)],[T_b_n{i}(3,4);T_b_n{i-1}(3,4)]);
   end
end

ee = plotTransforms(eval_T_b_ee(1:3,4)',rotm2quat(eval_T_b_ee(1:3, 1:3)), FrameSize=0.2);

axis equal;
grid on;
xlim([-0.5 0.8]);
ylim([-0.5 0.5]);
zlim([0 0.8]);
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Inverse kinematics
syms X_ee Y_ee Z_ee real;

inv_kinematics_problem = [
    X_ee == T_b_ee(1,4);
    Y_ee == T_b_ee(2,4);
    Z_ee == T_b_ee(3,4);
];

joints_constraints =        ...
    q1 > -pi & q1 < pi &    ...
    q2 >= -0.3 & q2 <= 0 &  ... 
    q3 > -pi & q3 < pi;

inv_kinematics_sol = solve(inv_kinematics_problem, joints_constraints, [q1, q2, q3], 'ReturnConditions', true, 'Real', true);

% Test ee cords on which perform the kinematic inversion
desired_ee_coords = [0.3 0.2 0.5];

disp('----------------------------------------------');
disp('Desired EE coords:');
disp(desired_ee_coords);

q1_sol = subs(inv_kinematics_sol.q1, [X_ee Y_ee Z_ee], desired_ee_coords);
q2_sol = subs(inv_kinematics_sol.q2, [X_ee Y_ee Z_ee], desired_ee_coords);
q3_sol = subs(inv_kinematics_sol.q3, [X_ee Y_ee Z_ee], desired_ee_coords);

conditions_sol = simplify(subs(inv_kinematics_sol.conditions, [X_ee Y_ee Z_ee], desired_ee_coords));

syms l z;

% the z value is calculated such that the condition from the solve result
% is satisfied
zVal = acos(0.3125*(400*desired_ee_coords(1)^2 + 400*desired_ee_coords(3)^2 - 120*desired_ee_coords(3) + 9)^(1/2) - 2.5000);

if ~isreal(zVal)
    disp('Kinematics inversion failed');
    return;
end

q1_sol = double(subs(q1_sol(1), [l z], [0 zVal]));
q2_sol = double(subs(q2_sol(1), [l z], [0 zVal]));
q3_sol = double(subs(q3_sol(1), [l z], [0 zVal]));
disp(cell2table({" " q1_sol q2_sol q3_sol}, "VariableNames", ["Inverse kinematics solutions", "q1", "q2", "q3"]));

config(1).JointPosition = q1_sol;
config(2).JointPosition = q2_sol;
config(3).JointPosition = q3_sol;

printPose(getTransform(robot,config,"ee"),double(subs(T_b_ee, [q1 q2 q3], [q1_sol q2_sol q3_sol])));
disp('----------------------------------------------');
%% Explicit inverse kinematics equations

% explicit expressions for the kinematics inversion
q3_explicit_IK = acos(0.3125*(400*X_ee^2 + 400*Z_ee^2 - 120*Z_ee + 9)^(1/2) - 2.5000);
q1_explicit_IK = 2*atan((8*(156.2500*X_ee^2 + 156.2500*Z_ee^2 - 46.8750*Z_ee + 3.5156)^(1/2) - 100*X_ee)/(100*Z_ee - 15));
q2_explicit_IK = 0.1600*sin(q3_explicit_IK) - Y_ee - 0.3000;

random_q_to_test = [
    (rand(5,1) .* (2 * pi)) - pi, -(rand(5,1) .* 0.3), (rand(5,1) .* (2 * pi)) - pi
];

disp('----------------------------------------------');
disp('Testing inverse kinematics...');

for i = 1:size(random_q_to_test,1)
    config(1).JointPosition = random_q_to_test(i,1);
    config(2).JointPosition = random_q_to_test(i,2);
    config(3).JointPosition = random_q_to_test(i,3);

    eePose = getTransform(robot,config,"ee");

    q1_temp_sol = double(subs(q1_explicit_IK,[X_ee, Y_ee, Z_ee],eePose(1:3,4)'));
    q2_temp_sol = double(subs(q2_explicit_IK,[X_ee, Y_ee, Z_ee],eePose(1:3,4)'));
    q3_temp_sol = double(subs(q3_explicit_IK,[X_ee, Y_ee, Z_ee],eePose(1:3,4)'));

    config(1).JointPosition = q1_temp_sol;
    config(2).JointPosition = q2_temp_sol;
    config(3).JointPosition = q3_temp_sol;

    eePose_IK = getTransform(robot,config,"ee");

    printPose(eePose,eePose_IK, true, false, [" ", "Robotics toolbox", "Inverse kinematics", "Delta"]);
end
disp('----------------------------------------------');
%% Jacobians

theta_euler = asin(-T_b_ee(3,1));
psi_euler = atan2(T_b_ee(2,1), T_b_ee(1,1));
phi_euler = atan2(T_b_ee(3,2) / cos(theta_euler), T_b_ee(3,3) / cos(theta_euler));

X_ee = T_b_ee(1,4);
Y_ee = T_b_ee(2,4);
Z_ee = T_b_ee(3,4);

% Analytical jacobian
Ja = [
    gradient(phi_euler, [q1 q2 q3])'; 
    gradient(theta_euler, [q1 q2 q3])';
    gradient(psi_euler, [q1 q2 q3])';
    gradient(X_ee, [q1 q2 q3])'; 
    gradient(Y_ee, [q1 q2 q3])';
    gradient(Z_ee, [q1 q2 q3])';
];

Ja = simplify(Ja, 'seconds', 10);

q1_test = 0;
q2_test = 0;
q3_test = 0;

% Extracting the geometric jacobian with the robotics toolbox
config = homeConfiguration(robot);

config(1).JointPosition = q1_test;
confif(2).JointPosition = q2_test;
config(3).JointPosition = q3_test;

Jg_RT = geometricJacobian(robot, config, 'ee');

z1 = DH_transforms{1};
z1 = z1(1:3, 3);

z2 = DH_transforms{1} * DH_transforms{2};
z2 = z2(1:3, 3);


z3 = DH_transforms{1} * DH_transforms{2} * DH_transforms{3};
z3 = z3(1:3, 3);

Jg = [
    z1 [0;0;0] z3;
    gradient(X_ee, [q1 q2 q3])'; 
    gradient(Y_ee, [q1 q2 q3])';
    gradient(Z_ee, [q1 q2 q3])';    
];

eval_Jg = double(subs(Jg, [q1 q2 q3], [q1_test q2_test q3_test]));

disp('----------------------------------------------');
disp('Difference between the two geometric jacobians');
delta = Jg_RT - eval_Jg
disp('----------------------------------------------');


