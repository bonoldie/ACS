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

env1K_low = 1;

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

freq = 0.5;
traj_A = 1;

traj_A_square = 3;

traj_adaptive = traj_A*sin(freq*2*pi*t);
dtraj_adaptive = diff(traj_adaptive);
ddtraj_adaptive = diff(dtraj_adaptive);

traj_adaptive = matlabFunction(traj_adaptive, Vars=t);
dtraj_adaptive = matlabFunction(dtraj_adaptive, Vars=t);
ddtraj_adaptive = matlabFunction(ddtraj_adaptive, Vars=t);

ddtraj_square_adaptive = @(t_i) double(traj_A_square*(square(t_i*freq*2*pi - (freq*pi))));

adaptive_q_i = -pi;
adaptive_dq_i = 0;

K_theta = inv(1);

Kd_adaptive = 10;

% Joint space trajectory

traj1(t) = [pi*sin(t) 0.2*sin(t) pi*cos(t)]';
traj1 = [traj1 diff(traj1) diff(diff(traj1))];

traj1 = matlabFunction(traj1, Vars=t);

%% force control
q_i_force = [pi/2;-0.2;-pi/5];

% x_f = [0; -0.7600; 0.5500];
f_d = [0 -1 0]';

M_d_force = diag([1 1 1]);

K_p_force = diag([0.1 10 0.1]);
K_d_force = diag([0.01 5 0.01]);

C_f_Kf = eye(myRobot.DOF)*5;
C_f_Ki = eye(myRobot.DOF)*3;

traj_force_position = @(t) [sin(t)*0.5500;-0.7600;cos(t)*0.5500];
traj_force_position = matlabFunction(traj_force_position, Vars=t);

arch_traj_force_position = @(t) [sin((sin(t/2) * pi/4))* 0.3 + 0.5500;-0.7600+ (t*0);cos((sin(t/2) * pi/4)) * 0.3 + 0.5500];
% sim = arch_traj_force_position(0:0.01:10);
% plot3(sim(1, :), sim(2, :), sim(3, :));
% axis equal;
arch_traj_force_position = matlabFunction(arch_traj_force_position, Vars=t);

envForceOrigin = env1Origin;
envForceNormal = env1Normal;
envForceK = 20;

envFriction = [0 0 0]';


%% impedance control

Md_impedance = eye(myRobot.DOF)*0.5*0.001;

Kp_impedance = eye(myRobot.DOF)*30*0.0001;
Kd_impedance = eye(myRobot.DOF)*1.5*0.0005;


traj_impedance_position = [(sin(t)/10);-0.7600 + (sin(t)/10);(cos(t)/10) + 0.55];

traj_impedance = matlabFunction(traj_impedance_position, Vars=t);
dtraj_impedance= matlabFunction(diff(traj_impedance_position), Vars=t);
ddtraj_impedance = matlabFunction(diff(diff(traj_impedance_position)), Vars=t);

%% admittance control
q_i_admittance = [pi/2+pi/8;0-0.05;-pi/2+pi/8];

Kp_motion_cotrol = eye(myRobot.DOF)*30;
Kd_motion_control = eye(myRobot.DOF)*1.5;

Kp_admittance = eye(myRobot.DOF)*5;
Kd_admittance = eye(myRobot.DOF)*1;

Md_admittance = eye(myRobot.DOF)*0.5;


traj_admittance_position = [(sin(t)/10);-0.7600 + (sin(t)/10);(cos(t)/10) + 0.55];

traj_admittance = matlabFunction(traj_admittance_position, Vars=t);
dtraj_admittance = matlabFunction(diff(traj_admittance_position), Vars=t);
ddtraj_admittance = matlabFunction(diff(diff(traj_admittance_position)), Vars=t);


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