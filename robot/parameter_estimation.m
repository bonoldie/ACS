close all;
clear all;
clc;

addpath(genpath("."));

% load the robot model 
load robot_model.mat;

% symbols init for joint variables
syms t real;
syms q1 q2 q3 real;
syms dq1 dq2 dq3 real;
syms ddq1 ddq2 ddq3 real;

% masses and inertias are now unknows(but fixed)
syms m1 m2 m3 real;

syms I1_1 I1_2 I1_3 I1_4 I1_5 I1_6 I2_1 I2_2 I2_3 I2_4 I2_5 I2_6  I3_1 I3_2 I3_3 I3_4 I3_5 I3_6 real;

q = [q1;q2;q3];
dq = [dq1;dq2;dq3];
ddq = [ddq1;ddq2;ddq3];

% masses
m = [0; m1; m2; m3; 0];

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

T_b_i = cumulateTransforms(T_dh);

CoM = {
    0 applyTransform(T_b_i{2}, [-delta_0_1/2;0;0]) applyTransform(T_b_i{3}, [0;delta_1_2/2;0]) applyTransform(T_b_i{4}, [-delta_2_3/2;0;0]) 0
};

I_Link1_CoM = inertiaVec2tensor([I1_1 I1_2 I1_3 I1_4 I1_5 I1_6]);
I_Link2_CoM = inertiaVec2tensor([I2_1 I2_2 I2_3 I2_4 I2_5 I2_6]);
I_Link3_CoM = inertiaVec2tensor([I3_1 I3_2 I3_3 I3_4 I3_5 I3_6]);

MoI_CoM = {zeros(3), I_Link1_CoM, I_Link2_CoM, I_Link3_CoM, zeros(3)};

I_Link1 = parallelAxis(I_Link1_CoM, m1, [-delta_0_1/2;0;0]);
I_Link2 = parallelAxis(I_Link2_CoM, m2, [0;0;-delta_1_2/2]);
I_Link3 = parallelAxis(I_Link3_CoM, m3, [-delta_2_3/2;0;0]);

MoI = {zeros(3), I_Link1, I_Link2, I_Link3, zeros(3)};

g =  [0;0;-9.81];

% Custom robotic structure, now in symbolic form
mySymbolicRobot = loadRobot(transforms, framesName, jointsType, linksName, m, MoI, CoM, [q dq ddq], g);

% parameters to estimate
params = [m1 I1_1 I1_2 I1_3 I1_4 I1_5 I1_6 m2 I2_1 I2_2 I2_3 I2_4 I2_5 I2_6  m3 I3_1 I3_2 I3_3 I3_4 I3_5 I3_6]';

% torques expression with symbolic masses and inertias
% tau = Y * params
tau = mySymbolicRobot.dynamics.B*mySymbolicRobot.jointsSymbol(:, 3)+mySymbolicRobot.dynamics.C*mySymbolicRobot.jointsSymbol(:,2) + mySymbolicRobot.dynamics.G;

% Extract the regressor Y from tau
Y = equationsToMatrix(tau, params);

%% Joint-space trajectory

Ti = 0;
Tf = 2;
steps = 200;

ts = (Ti:floor(Tf- Ti)/steps:Tf);

traj = [
    10*sin(t)*(1 + cos(4*t));
    5*cos(t)*(2 - cos(3*t));
    20*sin(t) - 8 * sin(2*t);
];

dtraj = diff(traj, t);
ddtraj = diff(dtraj, t);

% Evaluated each joint trajectory for each time step
traj_eval = double(subs(traj, t, ts));
dtraj_eval = double(subs(dtraj, t, ts));
ddtraj_eval = double(subs(ddtraj, t, ts));

% torques of the robot give a certain state
torques = zeros(size(traj_eval));

% 
Y_eval = zeros(size(traj_eval,1) * size(traj_eval,2), size(params, 1));

for i=1:size(traj_eval,2)
    currentJointState = num2cell([traj_eval(:, i);dtraj_eval(:, i);ddtraj_eval(:, i)]);
    torques(:, i) = myRobot.func.tau(currentJointState{:});
   
    Y_eval(((i-1)*size(traj_eval,1)) + 1: i*size(traj_eval,1),:) = subs(Y,[q;dq;ddq],[traj_eval(:, i);dtraj_eval(:, i);ddtraj_eval(:, i)] );
end

% Reshape torques to  column vector
torques_vec = reshape(torques, [1 prod(size(traj_eval))])';

estimation_evolution_by_time = { ...
    {1, [], zeros(size(traj_eval))},...
    {2, [], zeros(size(traj_eval))},...
    {3, [], zeros(size(traj_eval))},...
    {4, [], zeros(size(traj_eval))},...
    {5, [], zeros(size(traj_eval))},...
    {numel(ts), [], zeros(size(traj_eval))}
};

for i=1:size(estimation_evolution_by_time,2)
    estimatedParams = pinv(Y_eval(1:estimation_evolution_by_time{i}{1}*mySymbolicRobot.DOF, :)) * torques_vec(1:estimation_evolution_by_time{i}{1}*mySymbolicRobot.DOF);
    estimation_evolution_by_time{i}{2}  = estimatedParams;
    
    for j=1:numel(ts)
        estimation_evolution_by_time{i}{3}(:, j) = subs(tau,[params;q;dq;ddq], [ estimatedParams; traj_eval(:, j);dtraj_eval(:, j);ddtraj_eval(:, j)]);
    end
end


%% Plots

figure('Position',[10 10 1000 1000], "Renderer", "painters");

for jointN=1:mySymbolicRobot.DOF

    subplot(mySymbolicRobot.DOF, 1, jointN);
    hold on;
    
    for i=1:size(estimation_evolution_by_time,2)
        plot(ts, estimation_evolution_by_time{i}{3}(jointN, :), DisplayName=num2str(estimation_evolution_by_time{i}{1}),LineWidth=2);
    end
    plot(ts, torques(jointN, :), DisplayName='Real torque', LineWidth=2);
    xlabel('time (s)');
    ylabel('effort');
    title("Joint " + num2str(jointN));
    grid on;
    legend;
end

figure('Position',[10 10 1000 1000], "Renderer", "painters");

for jointN=1:mySymbolicRobot.DOF

    subplot(mySymbolicRobot.DOF, 1, jointN);
    hold on;
    
    for i=1:size(estimation_evolution_by_time,2)
        plot(ts, torques(jointN, :) - estimation_evolution_by_time{i}{3}(jointN, :), DisplayName=num2str(estimation_evolution_by_time{i}{1}),LineWidth=2);
        % legend(strcat(num2str(estimation_evolution_by_time{i}{1}),' estimation step'));
    end

    xlabel('time (s)');
    ylabel('effort error');
    title("Joint " + num2str(jointN));
    grid on;
    legend;
end