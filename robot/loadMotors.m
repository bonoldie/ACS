function [] = loadMotors(roboticStructure, motors)
    %LOADMOTORS Summary of this function goes here
    %   Detailed explanation goes here

    if size(motors, 2) ~= roboticStructure.DOF
        error('Number of motors must be equal to the robot DOF')
    end

    roboticStructure.motors = motors;
end