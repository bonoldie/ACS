function [motorStructure] = loadMotor(k,Inertia, rotationAxis)
    %LOADMOTORS Summary of this function goes here
    %   Detailed explanation goes here
    motorStructure.k = k;
    motorStructure.Inertia = Inertia;
    motorStructure.rotationAxis = rotationAxis;
end