function [out] = partialJg(kinematics, p_CoM, linkNumber)
%PARTIALJG Summary of this function goes here
%   Detailed explanation goes here

Tcumulate = cumulateTransforms(kinematics.T);

out = sym(zeros(6, kinematics.DOF));

p_li = p_CoM{linkNumber};

for j=1:linkNumber
    z_j_minus_1 = Tcumulate{j}(1:3,3);
    p_j_minus_1 = Tcumulate{j}(1:3,4);
    
    if strcmp(kinematics.joints.types{j}, 'Revolute')
        out(:, j) = [
            cross(z_j_minus_1, p_li - p_j_minus_1);
            z_j_minus_1
        ];
    else
        out(:, j) = [
            z_j_minus_1;
            0;0;0;
        ];
    end
end

end

