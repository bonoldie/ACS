function [out] = geometricJacobianTo(kinematics,  to)
%GEOMETRICJACOBIANTO Summary of this function goes here
%   Detailed explanation goes here

fromIndex = find(strcmp(kinematics.frames, '0'));

toIndex = to;

if ischar(to)
    toIndex = find(strcmp(kinematics.frames, to));
end

Tcumulate = cumulateTransforms(kinematics.T);

p_li = Tcumulate{toIndex-2}(1:3,4);

Jg = sym(zeros(6,3));

for j=1:toIndex-fromIndex
    z_j_minus_1 = Tcumulate{j}(1:3, 3);

    if strcmp(kinematics.joints.types{j}, 'Revolute')
        p_j_minus_1 = Tcumulate{j}(1:3, 4);
        Jg(1:3, j) = cross(z_j_minus_1, p_li - p_j_minus_1) ;
        Jg(4:6, j) = z_j_minus_1;
    else
        Jg(1:3, j) = z_j_minus_1;
    end
end

out = Jg;

end

