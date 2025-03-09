% Bootstrap the simulink environment
clear all;

addpath(genpath('.'));
load('robot_model.mat');
robot_tb = importrobot('RPR_yyz_renamed.urdf');
robot_tb.DataFormat = 'column';

q_i = [pi/2;0;pi/3];
q_i_up = [pi/2;0;-pi/8];

dq_i = [0;0;0];

env1Origin = [0;-0.4;0];
env1Normal = [0;1;0];
env1K = 50;

env2Origin = [0;-0.45;0];
env2Normal = [1;1;0];
env2K = 100;

env2K_low = 1;

q_d_compliance =  [pi/2+0.2;0;-pi/3];
% x_d = 


Kp = eye(myRobot.DOF)*5;
Kd = eye(myRobot.DOF)*1.25;
Ki = eye(myRobot.DOF);

Kp_op = diag([50 50 50 0.1 0.1 0.1]) .* 0.75;
Kd_op = diag([10 10 10 0.05 0.05 0.05]) .* 0.75;

% Adaptive control trajectory


adaptive_params = struct();
starting_params = struct();

link_mass = 2;
link_length = 0.5;

adaptive_params.G = link_mass * link_length * -9.81;
adaptive_params.I = 0.2;
adaptive_params.F = 0.2;

starting_params.G = adaptive_params.G * 0.25;
starting_params.I = adaptive_params.I * 0.25;
starting_params.F = adaptive_params.F * 0.25;

lambda = 2;

syms t real;

freq = 5;
traj_A = 0.5;

traj_A_square = 3;

traj_adaptive = traj_A*sin(freq*t);%* sin(cos(t*2) * freq * 5) + 2*cos(0.2*t);
dtraj_adaptive = diff(traj_adaptive);
ddtraj_adaptive = diff(dtraj_adaptive);

traj_adaptive = matlabFunction(traj_adaptive, Vars=t);
dtraj_adaptive = matlabFunction(dtraj_adaptive, Vars=t);
ddtraj_adaptive = matlabFunction(ddtraj_adaptive, Vars=t);

ddtraj_square_adaptive = @(t_i) double(traj_A_square*(square(t_i*freq)));

adaptive_q_i = -pi/2;
adaptive_dq_i = pi/10;

K_theta = 0.10;
K_d_adaptive = 3;

% Joint space trajectory

traj1(t) = [pi*sin(t) 0.2*sin(t) pi*cos(t)]';
traj1 = [traj1 diff(traj1) diff(diff(traj1))];

traj1 = matlabFunction(traj1, Vars=t);

% force control
q_i_force = [pi/2;0;-pi/2];

x_f = [0; -0.7600; 0.5500];
f_d = [0 -1 0];

M_d = diag([1 1 1]);

K_p_force = eye(myRobot.DOF)*5;
K_d_force = eye(myRobot.DOF)*1.25;

C_f_Kf = eye(myRobot.DOF)*5;
C_f_Ki = eye(myRobot.DOF)*0.5;


%% plots
figure();
show(toolboxRobot, q_d_compliance);
hold on;
envPlane = constantplane(env1Normal,sign(dot(env1Normal, env1Origin))*norm(env1Origin));
envPlane.FaceAlpha = 0.6;
quiver3(env1Origin(1), env1Origin(2), env1Origin(3), env1Normal(1)/10, env1Normal(2)/10, env1Normal(3)/10, 3, "filled", LineWidth=3, Color='b', MaxHeadSize=2);
plot3(env1Origin(1), env1Origin(2), env1Origin(3), ".", LineWidth=4);

grid off;
axis off;
zlim([-0.1 0.8]);
ylim([-0.7 0.3]);
xlim([-0.4 0.4]);

set(gca, 'CameraPosition',[6.9143    1.3053    3.6813]);
set(gca, 'CameraTarget',[ -0.0475   -0.0819    0.3859]);
set(gca, 'CameraViewAngle', 7.9999);


figure();
show(toolboxRobot, q_d_compliance);
hold on;
envPlane = constantplane(env2Normal,sign(dot(env2Normal, env2Origin))*norm(env2Origin));
envPlane.FaceAlpha = 0.6;
quiver3(env2Origin(1), env2Origin(2), env2Origin(3), env2Normal(1)/10, env2Normal(2)/10, env2Normal(3)/10, 3, "filled", LineWidth=3, Color='b', MaxHeadSize=2);
plot3(env2Origin(1), env2Origin(2), env2Origin(3), ".", LineWidth=4);

grid off;
axis off;
zlim([-0.1 0.8]);
ylim([-0.7 0.3]);
xlim([-0.4 0.4]);

set(gca, 'CameraPosition',[6.9143    1.3053    3.6813]);
set(gca, 'CameraTarget',[ -0.0475   -0.0819    0.3859]);
set(gca, 'CameraViewAngle', 7.9999);