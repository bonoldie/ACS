close all;
clear all;
clc;

% imports
addpath('utils/');

% loads robot from URDF
robot = importrobot('RPR_yyz ENRICO BONOLDI.urdf');
syms q1_dot q2_dot q3_dot;

links_mass = [1 1 1];

showdetails(robot);

% Kinetic energy

total_K = 0;

Link2_inertia_sigma_0 = cylinder_inertia(links_mass(1), 0.02, 0, 0.4);
Link3_inertia_sigma_0 = prism_inertia(links_mass(2), 0.3, 0.03, 0.03);
Link4_inertia_sigma_0 = cylinder_inertia(links_mass(3), 0.02, 0, 0.16);


robot.getBody("Link2").Mass = links_mass(1);
robot.getBody("Link2").Inertia = Link2_inertia_sigma_0;

robot.getBody("Link3").Mass = links_mass(2);
robot.getBody("Link3").Inertia = Link3_inertia_sigma_0;

robot.getBody("Link4").Mass = links_mass(3);
robot.getBody("Link4").Inertia = Link4_inertia_sigma_0;


config = homeConfiguration(robot);


geometricJacobian(robot, config, "ee")
geometricJacobian(robot, config, "Link3")
geometricJacobian(robot, config, "Link2")