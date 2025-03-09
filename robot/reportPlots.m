% clear all;
% close all;
clc;

load robot_model;
% plot robot in home configuration
figure();
show(toolboxRobot, homeConfiguration(toolboxRobot));
hold on;

grid off;
axis off;

set(gca,'CameraPosition',[9.9720   10.2682    9.7554]);
set(gca,'CameraTarget',[0.1822   -0.1650   -0.0073]);
set(gca,'CameraViewAngle',2.5142);

text(0,0,-0.1, '$\Sigma_b$' ,'fontSize', 20 ,  'Interpreter', 'latex');

text(0, 0, 0.30, '$\Sigma_0$' ,'fontSize', 20 ,  'Interpreter', 'latex');
text(0.4, 0, 0.30, '$\Sigma_1$' ,'fontSize', 20 ,  'Interpreter', 'latex');
text(0.4,  -0.3000,0.28, '$\Sigma_2$' ,'fontSize', 20 ,  'Interpreter', 'latex')
text(0.65,  -0.3000,0.28, '$\Sigma_3$' ,'fontSize', 20 ,  'Interpreter', 'latex');
text(0.65,  -0.3000,0.05, '$\Sigma_{ee}$' ,'fontSize', 20 ,  'Interpreter', 'latex');


%% IK 

figure();
% Robotics toolbox inverse kinematics
show(toolboxRobot, [0.7854; -0.1700; -0.3491]);
grid off;
axis off;
set(gca,'CameraPosition',[14.5703    8.1855    4.5313]);
set(gca,'CameraTarget',[  -0.0446   -0.0216    0.1668]);
set(gca,'CameraViewAngle',2.7934);

figure();
% my inverse kinematics
show(toolboxRobot, [0.7854; -0.0606; 0.3491]);
grid off;
axis off;
set(gca,'CameraPosition',[14.5703    8.1855    4.5313]);
set(gca,'CameraTarget',[  -0.0446   -0.0216    0.1668]);
set(gca,'CameraViewAngle',2.7934);

%% PD gravity compensation

q_d = [-pi/3 -0.18 -pi/3]';

figure();
show(toolboxRobot, q_d);
grid off;
axis off;

set(gca,'CameraPosition',[9.9720   10.2682    9.7554]);
set(gca,'CameraTarget',[0.1822   -0.1650   -0.0073]);
set(gca,'CameraViewAngle',2.5142);

%% PD gravity compensation - op. space

q_d = [-pi/3 -0.18 (3*pi)/2]';

figure();
show(toolboxRobot, q_d);
grid off;
axis off;

set(gca,'CameraPosition',[9.9720   10.2682    9.7554]);
set(gca,'CameraTarget',[0.1822   -0.1650   -0.0073]);
set(gca,'CameraViewAngle',2.5142);
