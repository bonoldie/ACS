function [Ja, Ta] = analyticalJac(roboticStructure)
%ANALYTICALJAC Summary of this function goes here
%   Detailed explanation goes here
Ja = sym(zeros(6, roboticStructure.DOF));

translation = roboticStructure.T_b_i{end}(1:3, 4);
rotation = roboticStructure.T_b_i{end}(1:3, 1:3);

phi = atan2(rotation(2, 3), rotation(1,3));
theta = atan2(sqrt(rotation(1, 3)^2 + rotation(2, 3)^2), rotation(3,3));
psi = atan2(rotation(3, 2), -rotation(3,1));

for i=1:roboticStructure.DOF
    Ja(1:3, i) = diff(translation, roboticStructure.jointsSymbol(i, 1));

    Ja(4, i) = diff(phi, roboticStructure.jointsSymbol(i, 1));
    Ja(5, i) = diff(theta, roboticStructure.jointsSymbol(i, 1));
    Ja(6, i) = diff(psi, roboticStructure.jointsSymbol(i, 1));
end 

Ta = sym(eye(6));
Ta(4:6,4:6) = [
    0 -sin(phi) cos(phi)*sin(theta);
    0 cos(phi)  sin(phi)*sin(theta);
    1 0         cos(theta); 
];
end

