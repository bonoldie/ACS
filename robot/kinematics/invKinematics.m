function [q] = invKinematics(pose)
arguments
    pose (3, 1) double  
end

Xee = pose(1);
Yee = pose(2);
Zee = pose(3);

q = zeros(3,1);

q(3) = acos(0.3125*(400*Xee^2 + 400*Zee^2 - 120*Zee + 9)^(1/2) - 2.5000);
q(1) = 2*atan((8*(156.2500*Xee^2 + 156.2500*Zee^2 - 46.8750*Zee + 3.5156)^(1/2) - 100*Xee)/(100*Zee - 15));
q(2) = -0.3000 + 0.1600*sin(q(3)) - Yee;

if ~isreal(q(3))
    disp('Inverse kinematics error, is the pose reachable?');
end

% q(1) = asin((Zee - 0.15) / (Xee));
% q(3) = acos((Xee - 0.4)/ 0.16);
% q(2) = -0.3 + 0.16 * sin(q(3)) - Yee;


end

